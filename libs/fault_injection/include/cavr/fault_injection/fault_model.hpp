#pragma once

// Fault injection for the virtual robot: the deterministic library that gives a
// simulated controller the failure behaviour a real one has. It is pure and
// adapter-free — no Qt, no sockets, no clock of its own — so a scenario is driven
// by the same poll(now) ticks the robot runs on and replays bit-for-bit given a
// seed. The VirtualRobot owns one FaultInjector and applies its verdict each tick.
//
// Two kinds of fault:
//   * discrete trips — an E-stop, a servo/overload fault, a lost weld arc — that
//     fire once on a trigger (a time, a program step, or a seeded rate) and change
//     the controller's state (abort, servo error, arc down);
//   * continuous perturbations — encoder noise on the reported joints, a slow TCP
//     calibration drift — that colour every telemetry frame without stopping the run.
//
// Triggers and continuous noise draw from two independent RNG streams, so adding
// noise to a scenario does not shift when its timed/rate trips fire.

#include <cavr/core/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace cavr::fault_injection {

// SplitMix64: a tiny, portable PRNG. It is fully specified by its state, so the
// same seed yields the same stream on every platform — reproducibility is the
// whole point of a fault scenario (a "flaky" fault would be useless).
class DeterministicRng final {
 public:
  explicit DeterministicRng(std::uint64_t seed = 0x9E3779B97F4A7C15ull) : state_(seed) {}

  void reseed(std::uint64_t seed) noexcept {
    state_ = seed;
    has_spare_ = false;
  }

  [[nodiscard]] std::uint64_t next_u64() noexcept {
    state_ += 0x9E3779B97F4A7C15ull;
    std::uint64_t z = state_;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
  }

  // Uniform in [0, 1) using the top 53 bits (double mantissa width).
  [[nodiscard]] double uniform() noexcept {
    return static_cast<double>(next_u64() >> 11) * (1.0 / 9007199254740992.0);
  }

  // Standard normal via Box-Muller, caching the paired sample.
  [[nodiscard]] double normal() noexcept {
    if (has_spare_) {
      has_spare_ = false;
      return spare_;
    }
    double u1 = uniform();
    if (u1 < 1e-12) u1 = 1e-12;  // avoid log(0)
    const double u2 = uniform();
    const double mag = std::sqrt(-2.0 * std::log(u1));
    spare_ = mag * std::sin(2.0 * kPi * u2);
    has_spare_ = true;
    return mag * std::cos(2.0 * kPi * u2);
  }

 private:
  static constexpr double kPi = 3.14159265358979323846;
  std::uint64_t state_;
  bool has_spare_{false};
  double spare_{0.0};
};

enum class FaultKind {
  EmergencyStop,     // external E-stop: program aborts, servo -> Error, Critical
  ServoFault,        // drive / overload fault: program aborts, servo -> Error
  ArcLoss,           // welding arc drops mid-weld: weld signals fall, Warning
  EncoderNoise,      // continuous jitter on reported joint positions (rad std-dev)
  CalibrationDrift,  // continuous slow TCP drift (m/s along a fixed direction)
};

enum class TriggerKind {
  AtTime,  // once, when program-elapsed first reaches at_time_s
  AtStep,  // once, when execution enters command index at_step
  Rate,    // stochastic: mean rate_per_s occurrences per second (seeded)
  Always,  // no discrete trip; a continuous-effect spec (noise / drift)
};

// One fault the injector may raise. A continuous effect (EncoderNoise,
// CalibrationDrift) uses TriggerKind::Always and `magnitude`; a discrete fault uses
// one of AtTime / AtStep / Rate.
struct FaultSpec final {
  FaultKind kind{FaultKind::EmergencyStop};
  TriggerKind trigger{TriggerKind::AtTime};
  double at_time_s{0.0};                       // AtTime
  int at_step{-1};                             // AtStep (command index)
  double rate_per_s{0.0};                      // Rate (mean occurrences / second)
  double magnitude{0.0};                       // noise std-dev (rad) / drift speed (m/s)
  core::Vec3 direction{0.0, 0.0, 1.0};         // CalibrationDrift direction (tool frame)
  std::string label;                           // optional human message
};

// A fault that fired on a given tick, for the robot to act on and log.
struct FaultTrip final {
  FaultKind kind{FaultKind::EmergencyStop};
  std::string message;
};

// Convenience constructors for the common scenarios.
[[nodiscard]] inline FaultSpec estop_at(double time_s, std::string label = "Emergency stop") {
  FaultSpec s;
  s.kind = FaultKind::EmergencyStop;
  s.trigger = TriggerKind::AtTime;
  s.at_time_s = time_s;
  s.label = std::move(label);
  return s;
}

[[nodiscard]] inline FaultSpec servo_fault_at_step(int step, std::string label = "Servo overload") {
  FaultSpec s;
  s.kind = FaultKind::ServoFault;
  s.trigger = TriggerKind::AtStep;
  s.at_step = step;
  s.label = std::move(label);
  return s;
}

[[nodiscard]] inline FaultSpec arc_loss_at(double time_s, std::string label = "Weld arc lost") {
  FaultSpec s;
  s.kind = FaultKind::ArcLoss;
  s.trigger = TriggerKind::AtTime;
  s.at_time_s = time_s;
  s.label = std::move(label);
  return s;
}

[[nodiscard]] inline FaultSpec encoder_noise(double stddev_rad) {
  FaultSpec s;
  s.kind = FaultKind::EncoderNoise;
  s.trigger = TriggerKind::Always;
  s.magnitude = stddev_rad;
  s.label = "Encoder noise";
  return s;
}

[[nodiscard]] inline FaultSpec calibration_drift(double speed_m_s, core::Vec3 direction) {
  FaultSpec s;
  s.kind = FaultKind::CalibrationDrift;
  s.trigger = TriggerKind::Always;
  s.magnitude = speed_m_s;
  s.direction = direction;
  s.label = "Calibration drift";
  return s;
}

// Holds the armed fault scenario and decides, each tick, what fires. Owned by the
// VirtualRobot; nothing here reaches out to a clock or a socket.
class FaultInjector final {
 public:
  explicit FaultInjector(std::uint64_t seed = 1) : seed_(seed) { arm(); }

  void add(FaultSpec spec) {
    specs_.push_back(std::move(spec));
    fired_.push_back(false);
  }
  void clear() {
    specs_.clear();
    fired_.clear();
  }
  [[nodiscard]] bool empty() const noexcept { return specs_.empty(); }
  [[nodiscard]] std::size_t size() const noexcept { return specs_.size(); }
  [[nodiscard]] const std::vector<FaultSpec>& specs() const noexcept { return specs_; }

  void set_seed(std::uint64_t seed) noexcept { seed_ = seed; }
  [[nodiscard]] std::uint64_t seed() const noexcept { return seed_; }

  // Reset fired-state and both RNG streams to the seed. Called when a program
  // (re)starts, so the same scenario reproduces run to run.
  void arm() {
    std::fill(fired_.begin(), fired_.end(), false);
    trigger_rng_.reseed(seed_);
    noise_rng_.reseed(seed_ ^ 0xD1B54A32D192ED03ull);  // decorrelated from triggers
  }

  // Discrete faults tripped on this tick. `elapsed_s` is program time, `dt_s` the
  // tick delta (used by the Rate trigger), `step` the current command index and
  // `prev_step` the previous tick's step (for AtStep edge detection).
  [[nodiscard]] std::vector<FaultTrip> poll(double elapsed_s, double dt_s, int step, int prev_step) {
    std::vector<FaultTrip> trips;
    for (std::size_t i = 0; i < specs_.size(); ++i) {
      const FaultSpec& s = specs_[i];
      if (s.trigger == TriggerKind::Always) continue;
      // AtTime / AtStep are one-shot; Rate models a recurring event and re-fires.
      if (s.trigger != TriggerKind::Rate && fired_[i]) continue;
      bool fire = false;
      switch (s.trigger) {
        case TriggerKind::AtTime:
          fire = elapsed_s >= s.at_time_s;
          break;
        case TriggerKind::AtStep:
          fire = step == s.at_step && prev_step != s.at_step;
          break;
        case TriggerKind::Rate:
          fire = dt_s > 0.0 && trigger_rng_.uniform() < s.rate_per_s * dt_s;
          break;
        case TriggerKind::Always:
          break;
      }
      if (fire) {
        if (s.trigger != TriggerKind::Rate) fired_[i] = true;
        trips.push_back({s.kind, message_for(s)});
      }
    }
    return trips;
  }

  // Add encoder jitter to the reported joints (sum of all EncoderNoise specs). Draws
  // from the noise stream, so it never perturbs the timing of the trigger stream.
  void perturb_joints(std::vector<double>& q) {
    for (const FaultSpec& s : specs_) {
      if (s.kind != FaultKind::EncoderNoise || s.magnitude <= 0.0) continue;
      for (double& v : q) v += noise_rng_.normal() * s.magnitude;
    }
  }

  // Accumulated TCP offset from calibration drift at `elapsed_s` (sum of all
  // CalibrationDrift specs). Deterministic (no RNG): a ramp along each direction.
  [[nodiscard]] core::Vec3 tcp_drift(double elapsed_s) const {
    core::Vec3 d{0.0, 0.0, 0.0};
    for (const FaultSpec& s : specs_) {
      if (s.kind != FaultKind::CalibrationDrift) continue;
      const double n = std::sqrt(s.direction.x_m * s.direction.x_m +
                                 s.direction.y_m * s.direction.y_m +
                                 s.direction.z_m * s.direction.z_m);
      if (n < 1e-9) continue;
      const double scale = s.magnitude * elapsed_s / n;
      d.x_m += s.direction.x_m * scale;
      d.y_m += s.direction.y_m * scale;
      d.z_m += s.direction.z_m * scale;
    }
    return d;
  }

  [[nodiscard]] bool has_continuous() const noexcept {
    for (const FaultSpec& s : specs_)
      if (s.kind == FaultKind::EncoderNoise || s.kind == FaultKind::CalibrationDrift) return true;
    return false;
  }

 private:
  [[nodiscard]] static std::string message_for(const FaultSpec& s) {
    if (!s.label.empty()) return s.label;
    switch (s.kind) {
      case FaultKind::EmergencyStop: return "Emergency stop";
      case FaultKind::ServoFault: return "Servo fault";
      case FaultKind::ArcLoss: return "Weld arc lost";
      case FaultKind::EncoderNoise: return "Encoder noise";
      case FaultKind::CalibrationDrift: return "Calibration drift";
    }
    return "Fault";
  }

  std::vector<FaultSpec> specs_;
  std::vector<bool> fired_;
  std::uint64_t seed_;
  DeterministicRng trigger_rng_;
  DeterministicRng noise_rng_;
};

}  // namespace cavr::fault_injection
