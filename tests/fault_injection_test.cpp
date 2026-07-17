// FaultInjector: the deterministic fault library the virtual robot runs on. These
// tests pin the two guarantees that make a fault scenario useful — it reproduces
// bit-for-bit from a seed, and each trigger (time / step / rate) fires exactly when
// and as often as specified — plus the continuous effects (encoder noise, TCP
// drift). No robot, no clock: the injector is driven directly.

#include <cavr/fault_injection/fault_model.hpp>

#include <cmath>
#include <iostream>
#include <string_view>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    ++failures;
  }
}

namespace fault = cavr::fault_injection;

// Same seed → same stream; different seed → different stream.
void test_rng_determinism() {
  fault::DeterministicRng a(12345), b(12345), c(99);
  bool same = true, differ = false;
  for (int i = 0; i < 64; ++i) {
    const std::uint64_t va = a.next_u64();
    if (va != b.next_u64()) same = false;
    if (va != c.next_u64()) differ = true;
  }
  check(same, "identical seeds produce identical streams");
  check(differ, "different seeds produce different streams");

  // uniform() stays in [0,1); normal() has ~zero mean over many draws.
  fault::DeterministicRng r(7);
  bool in_range = true;
  double sum = 0.0;
  constexpr int kN = 20000;
  for (int i = 0; i < kN; ++i) {
    const double u = r.uniform();
    if (u < 0.0 || u >= 1.0) in_range = false;
    sum += r.normal();
  }
  check(in_range, "uniform() stays within [0, 1)");
  check(std::abs(sum / kN) < 0.05, "normal() has approximately zero mean");
}

// An AtTime fault fires once, on the first tick at/after its time, and never again.
void test_at_time_trigger() {
  fault::FaultInjector inj(1);
  inj.add(fault::estop_at(2.0));
  inj.arm();

  int fires = 0;
  int prev_step = -1;
  for (int i = 0; i <= 50; ++i) {  // 0.0 .. 5.0 s in 0.1 s ticks
    const double t = i * 0.1;
    const auto trips = inj.poll(t, 0.1, 0, prev_step);
    for (const auto& trip : trips) {
      check(trip.kind == fault::FaultKind::EmergencyStop, "AtTime trip is the E-stop");
      check(t >= 2.0 - 1e-9, "E-stop does not fire before its time");
      ++fires;
    }
  }
  check(fires == 1, "AtTime fault fires exactly once");
}

// An AtStep fault fires on entering its command index, on the edge only.
void test_at_step_trigger() {
  fault::FaultInjector inj(1);
  inj.add(fault::servo_fault_at_step(2));
  inj.arm();

  const int steps[] = {0, 0, 1, 1, 2, 2, 2, 3};  // enters step 2 once
  int fires = 0;
  int prev = -1;
  double t = 0.0;
  for (int step : steps) {
    for (const auto& trip : inj.poll(t, 0.1, step, prev)) {
      check(trip.kind == fault::FaultKind::ServoFault, "AtStep trip is the servo fault");
      ++fires;
    }
    prev = step;
    t += 0.1;
  }
  check(fires == 1, "AtStep fault fires once on the step edge");
}

// A Rate fault fires a plausible number of times over a window, and reproducibly.
void test_rate_trigger_reproducible() {
  auto run = []() {
    fault::FaultInjector inj(42);
    fault::FaultSpec s;
    s.kind = fault::FaultKind::ArcLoss;
    s.trigger = fault::TriggerKind::Rate;
    s.rate_per_s = 1.0;  // mean 1 / s
    inj.add(s);
    inj.arm();
    int fires = 0;
    int prev = -1;
    for (int i = 0; i < 1000; ++i)  // 10 s of 0.01 s ticks
      fires += static_cast<int>(inj.poll(i * 0.01, 0.01, 0, prev).size());
    return fires;
  };
  const int a = run();
  const int b = run();
  check(a == b, "Rate trigger is reproducible for a fixed seed");
  check(a >= 3 && a <= 20, "Rate ~1/s fires a plausible count over 10 s");  // mean ~10
}

// arm() rewinds the scenario: a fired one-shot fault fires again after re-arming.
void test_arm_rewinds() {
  fault::FaultInjector inj(1);
  inj.add(fault::estop_at(1.0));
  inj.arm();
  int first = 0, prev = -1;
  for (int i = 0; i <= 20; ++i) first += static_cast<int>(inj.poll(i * 0.1, 0.1, 0, prev).size());
  check(first == 1, "fires once in the first run");

  inj.arm();  // operator reset
  int second = 0;
  for (int i = 0; i <= 20; ++i) second += static_cast<int>(inj.poll(i * 0.1, 0.1, 0, prev).size());
  check(second == 1, "fires again after re-arming");
}

// Encoder noise perturbs joints with the configured magnitude, and reproducibly;
// zero magnitude is a no-op.
void test_encoder_noise() {
  fault::FaultInjector inj(5);
  inj.add(fault::encoder_noise(0.01));  // 0.01 rad std-dev
  inj.arm();

  const std::vector<double> home(6, 0.0);
  std::vector<double> q = home;
  double sq = 0.0;
  int n = 0;
  for (int i = 0; i < 500; ++i) {
    q = home;
    inj.perturb_joints(q);
    for (double v : q) { sq += v * v; ++n; }
  }
  const double rms = std::sqrt(sq / n);
  check(std::abs(rms - 0.01) < 0.003, "encoder-noise RMS matches the configured std-dev");

  // Reproducible: same seed, same sequence of perturbations.
  fault::FaultInjector a(5), b(5);
  a.add(fault::encoder_noise(0.01));
  b.add(fault::encoder_noise(0.01));
  a.arm();
  b.arm();
  std::vector<double> qa(6, 0.0), qb(6, 0.0);
  a.perturb_joints(qa);
  b.perturb_joints(qb);
  bool same = true;
  for (std::size_t i = 0; i < qa.size(); ++i) if (std::abs(qa[i] - qb[i]) > 1e-15) same = false;
  check(same, "encoder noise is reproducible for a fixed seed");

  // No noise specs → joints untouched.
  fault::FaultInjector none(1);
  std::vector<double> clean(6, 0.0);
  none.perturb_joints(clean);
  bool untouched = true;
  for (double v : clean) if (v != 0.0) untouched = false;
  check(untouched, "no noise spec leaves joints untouched");
}

// Calibration drift is a deterministic ramp along its direction, growing with time.
void test_calibration_drift() {
  fault::FaultInjector inj(1);
  inj.add(fault::calibration_drift(0.001, cavr::core::Vec3{0.0, 0.0, 1.0}));  // 1 mm/s in +Z
  inj.arm();

  const auto d0 = inj.tcp_drift(0.0);
  check(std::abs(d0.z_m) < 1e-12, "no drift at t=0");
  const auto d5 = inj.tcp_drift(5.0);
  check(std::abs(d5.z_m - 0.005) < 1e-9, "drift is speed * time along the unit direction");
  check(std::abs(d5.x_m) < 1e-12 && std::abs(d5.y_m) < 1e-12, "drift stays on its axis");
}

}  // namespace

int main() {
  test_rng_determinism();
  test_at_time_trigger();
  test_at_step_trigger();
  test_rate_trigger_reproducible();
  test_arm_rewinds();
  test_encoder_noise();
  test_calibration_drift();

  if (failures != 0) {
    std::cerr << failures << " fault injection test(s) failed\n";
    return 1;
  }
  std::cout << "fault injection tests passed\n";
  return 0;
}
