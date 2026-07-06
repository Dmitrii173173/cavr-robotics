#pragma once

// Scalar velocity profile over a 1-D path coordinate s in [0, length]. This is
// the core of realistic motion: instead of teleporting along at a constant speed
// (a velocity step at every start/stop), the profile ramps the speed up and down
// under acceleration (trapezoidal) or under bounded jerk (S-curve), so a move has
// a real accel/cruise/decel shape.
//
// It is pure and unit-agnostic: s, v, a are expressed in whatever path unit the
// caller uses (radians for a joint sweep, millimetres for a Cartesian move). The
// same profile therefore serves both execution (VirtualRobot) and validation
// (cycle-time), which is the point of extracting it.
//
// Both shapes are represented as a sequence of constant-jerk phases; sampling is
// the exact cubic integral of jerk, so position is C1 (S-curve is C2) and the
// guarantees the tests assert hold analytically: s is monotonic non-decreasing,
// |v| <= v_max, |a| <= a_max, s(0)=0 and s(duration)=length.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace cavr::motion {

struct ProfileSample final {
  double s{0.0};  // position along the path
  double v{0.0};  // speed (ds/dt)
  double a{0.0};  // acceleration (dv/dt)
};

// A time-parameterised speed profile built from constant-jerk phases.
class VelocityProfile final {
 public:
  VelocityProfile() = default;

  // Seed the starting velocity (for a profile that begins mid-move at a blended
  // corner). Must be called before the first append().
  void set_initial_velocity(double v0) noexcept {
    if (phases_.empty()) { cur_v_ = v0; end_v_ = v0; }
  }

  [[nodiscard]] double duration() const noexcept { return total_; }
  [[nodiscard]] double length() const noexcept { return length_; }
  [[nodiscard]] double end_velocity() const noexcept { return end_v_; }
  [[nodiscard]] bool empty() const noexcept { return phases_.empty(); }

  // Sample at time t. t is clamped to [0, duration]; before the start the profile
  // sits at s=0, after the end it holds at s=length with the terminal velocity.
  [[nodiscard]] ProfileSample sample(double t) const {
    if (phases_.empty()) return {0.0, 0.0, 0.0};
    if (t <= 0.0) return {0.0, phases_.front().v0, phases_.front().a0};
    if (t >= total_) return {length_, end_v_, 0.0};
    std::size_t k = 0;
    while (k + 1 < phases_.size() && t >= phases_[k + 1].t0) ++k;
    const Phase& p = phases_[k];
    const double dt = t - p.t0;
    return {p.s0 + p.v0 * dt + 0.5 * p.a0 * dt * dt + p.j * dt * dt * dt / 6.0,
            p.v0 + p.a0 * dt + 0.5 * p.j * dt * dt,
            p.a0 + p.j * dt};
  }

  // Append a constant-jerk phase of length `dur`, continuing from the running end
  // state. `a_override` (if finite) sets the accel at the phase start — the step
  // that makes a trapezoidal (infinite-jerk) corner; leave NaN for a continuous
  // (S-curve) phase.
  void append(double dur, double jerk, double a_override = std::numeric_limits<double>::quiet_NaN()) {
    if (dur <= 0.0) return;
    const double s0 = cur_s_;
    const double v0 = cur_v_;
    const double a0 = std::isnan(a_override) ? cur_a_ : a_override;
    phases_.push_back({total_, s0, v0, a0, jerk, dur});
    total_ += dur;
    cur_s_ = s0 + v0 * dur + 0.5 * a0 * dur * dur + jerk * dur * dur * dur / 6.0;
    cur_v_ = v0 + a0 * dur + 0.5 * jerk * dur * dur;
    cur_a_ = a0 + jerk * dur;
    length_ = cur_s_;
    end_v_ = cur_v_;
  }

 private:
  struct Phase final {
    double t0, s0, v0, a0, j, dur;
  };
  std::vector<Phase> phases_;
  double total_{0.0};
  double length_{0.0};
  double end_v_{0.0};
  double cur_s_{0.0};  // running end state, so append() stays O(1) and exact
  double cur_v_{0.0};
  double cur_a_{0.0};
};

// Trapezoidal (bounded-acceleration) profile over `length`, honouring v_max and
// a_max, with optional non-zero boundary speeds v0/v1 (used when a corner is
// blended and the move does not come to rest). Acceleration is piecewise
// constant (+a_max, 0, -a_max); the peak speed is capped so the whole move fits
// in `length`.
[[nodiscard]] inline VelocityProfile plan_trapezoidal(double length, double v_max, double a_max,
                                                      double v0 = 0.0, double v1 = 0.0) {
  VelocityProfile prof;
  if (length <= 1e-12 || v_max <= 0.0 || a_max <= 0.0) return prof;
  v0 = std::clamp(v0, 0.0, v_max);
  v1 = std::clamp(v1, 0.0, v_max);

  // Peak speed of a symmetric accel/decel with no cruise: v_p^2 = a*L + (v0^2+v1^2)/2.
  double vp = std::sqrt(std::max(0.0, a_max * length + 0.5 * (v0 * v0 + v1 * v1)));
  // A boundary speed the segment is too short to reach is clamped down so the
  // profile stays feasible (the trajectory planner also caps these upstream).
  if (vp < v0) v0 = vp;
  if (vp < v1) v1 = vp;

  double cruise = 0.0;
  if (vp >= v_max) {  // trapezoid: cap at v_max and add a cruise phase
    vp = v_max;
    const double d_acc = (vp * vp - v0 * v0) / (2.0 * a_max);
    const double d_dec = (vp * vp - v1 * v1) / (2.0 * a_max);
    cruise = std::max(0.0, length - d_acc - d_dec);
  }

  const double t_acc = (vp - v0) / a_max;
  const double t_dec = (vp - v1) / a_max;
  const double t_cruise = vp > 1e-12 ? cruise / vp : 0.0;

  prof.set_initial_velocity(v0);
  prof.append(t_acc, 0.0, +a_max);   // ramp v0 -> vp
  prof.append(t_cruise, 0.0, 0.0);   // cruise at vp
  prof.append(t_dec, 0.0, -a_max);   // ramp vp -> v1
  return prof;
}

// Jerk-limited "double-S" profile, rest-to-rest. Honours v_max, a_max and j_max.
// Falls back to a trapezoid when the move is too short for the cruise speed to
// develop (the branch where v_max is not reached), which keeps the shape smooth
// and the code honest rather than papering over a degenerate solve.
[[nodiscard]] inline VelocityProfile plan_s_curve(double length, double v_max, double a_max,
                                                  double j_max) {
  if (length <= 1e-12 || v_max <= 0.0 || a_max <= 0.0 || j_max <= 0.0) {
    return plan_trapezoidal(length, v_max, a_max);
  }

  // Accel phase shape: does it reach a_max?  v_max*j_max >= a_max^2  <=>  yes.
  double t_j, t_a;  // jerk sub-phase, and total accel-phase duration
  if (v_max * j_max >= a_max * a_max) {
    t_j = a_max / j_max;
    t_a = t_j + v_max / a_max;  // area under the accel trapezoid == v_max
  } else {
    t_j = std::sqrt(v_max / j_max);  // triangular accel, a_max not reached
    t_a = 2.0 * t_j;
  }
  const double t_const_a = t_a - 2.0 * t_j;  // constant-a_max sub-phase (>=0)

  // Distance covered accelerating 0->v_max and decelerating v_max->0 is v_max*t_a
  // (each ramp averages v_max/2 over t_a). Need a cruise phase for the rest.
  const double d_ramps = v_max * t_a;
  if (d_ramps > length + 1e-12) {
    // v_max not reached over this distance — fall back to a smooth trapezoid.
    return plan_trapezoidal(length, v_max, a_max);
  }
  const double t_v = (length - d_ramps) / v_max;

  const double a_reached = std::min(a_max, j_max * t_j);
  VelocityProfile prof;
  prof.append(t_j, +j_max);            // 1: jerk up, a: 0 -> a_reached
  prof.append(t_const_a, 0.0);         // 2: hold a_max
  prof.append(t_j, -j_max);            // 3: jerk down, a -> 0, v = v_max
  prof.append(t_v, 0.0);               // 4: cruise
  prof.append(t_j, -j_max);            // 5: jerk down, a -> -a_reached
  prof.append(t_const_a, 0.0);         // 6: hold -a_max
  prof.append(t_j, +j_max);            // 7: jerk up, a -> 0, v = 0
  (void)a_reached;
  return prof;
}

// Dispatch: a jerk-limited profile for a rest-to-rest move when j_max is set,
// otherwise (or for a move with non-zero boundary speeds) a trapezoid.
[[nodiscard]] inline VelocityProfile plan_profile(double length, double v_max, double a_max,
                                                  double j_max = 0.0, double v0 = 0.0,
                                                  double v1 = 0.0) {
  if (j_max > 0.0 && v0 == 0.0 && v1 == 0.0) return plan_s_curve(length, v_max, a_max, j_max);
  return plan_trapezoidal(length, v_max, a_max, v0, v1);
}

}  // namespace cavr::motion
