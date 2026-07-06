#pragma once

// Time-parameterised joint-space trajectory: a sequence of segments, each with a
// velocity profile over its own path, laid end to end on a single clock. sample(t)
// returns the joint vector at time t. This is what an executor (VirtualRobot) or a
// validator (cycle time) runs; it knows nothing about controllers, Qt or sockets.
//
// Corner blending: when a segment declares a blend_radius, it does not decelerate
// to rest at its end — the boundary carries a matched non-zero speed into the next
// segment (so |velocity| stays continuous through the corner) and the joint path
// within blend_radius of the shared knot is replaced by a quadratic Bézier that
// cuts across the corner (X on the incoming leg -> knot control point -> Y on the
// outgoing leg). The deviation from the exact corner is bounded by blend_radius.

#include <cavr/motion/limits.hpp>
#include <cavr/motion/profile.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace cavr::motion {

// One planning unit handed to the trajectory planner. A MoveJ is two knots; a
// MoveL is two IK'd knots; a MoveC/arc is many knots with real chord lengths in
// knot_s. A dwell (Wait/ToolOn/...) is a single knot with hold_s > 0.
struct PlanSegment final {
  std::vector<std::vector<double>> knots;  // joint waypoints, size K >= 1
  std::vector<double> knot_s;              // cumulative path coord per knot (knot_s[0] == 0)
  double speed{0.0};                       // cruise speed cap in path units/s (> 0 for a move)
  double blend_radius{0.0};                // corner blend into the next segment (path units)
  double hold_s{0.0};                      // dwell duration for a stationary segment
  bool motion{true};                       // false => stationary step (no path travel)
  int command_index{0};                    // maps back to the task command / program line

  [[nodiscard]] double length() const noexcept { return knot_s.empty() ? 0.0 : knot_s.back(); }
};

class Trajectory final {
 public:
  Trajectory() = default;

  [[nodiscard]] bool empty() const noexcept { return segments_.empty(); }
  [[nodiscard]] std::size_t dof() const noexcept { return dof_; }
  [[nodiscard]] double duration() const noexcept { return total_; }
  [[nodiscard]] std::size_t segment_count() const noexcept { return segments_.size(); }

  // Joint vector at time t (clamped to [0, duration]). Before the start it holds
  // the first knot; after the end it holds the last knot.
  [[nodiscard]] std::vector<double> sample(double t) const {
    if (segments_.empty()) return std::vector<double>(dof_, 0.0);
    t = std::clamp(t, 0.0, total_);
    const std::size_t i = active_segment(t);
    const Seg& s = segments_[i];
    const double sp = seg_s(s, t);

    // Corner blend at the end of this segment (into i+1).
    for (const Blend& b : blends_) {
      if (b.before == i && sp >= s.length - b.r) {
        const double u = 0.5 * (sp - (s.length - b.r)) / b.r;  // [0, 0.5]
        return bezier(b, std::clamp(u, 0.0, 0.5));
      }
      if (b.after == i && sp <= b.r) {  // corner blend at the start of this segment
        const double u = 0.5 + 0.5 * sp / b.r;  // [0.5, 1]
        return bezier(b, std::clamp(u, 0.5, 1.0));
      }
    }
    return geom(s, sp);
  }

  // Index into the PlanSegment list active at time t — for mapping back to the
  // task command (step events, weld flags), which the executor tracks in parallel.
  [[nodiscard]] int segment_at(double t) const noexcept {
    if (segments_.empty()) return -1;
    return static_cast<int>(active_segment(std::clamp(t, 0.0, total_)));
  }

  [[nodiscard]] int command_at(double t) const noexcept {
    const int seg = segment_at(t);
    return seg < 0 ? -1 : segments_[static_cast<std::size_t>(seg)].command_index;
  }

  [[nodiscard]] bool segment_is_motion(int seg) const noexcept {
    return seg >= 0 && seg < static_cast<int>(segments_.size()) &&
           segments_[static_cast<std::size_t>(seg)].motion;
  }

  [[nodiscard]] const std::vector<double>& first_knot() const noexcept {
    static const std::vector<double> empty;
    if (segments_.empty() || segments_.front().knots.empty()) return empty;
    return segments_.front().knots.front();
  }
  [[nodiscard]] const std::vector<double>& last_knot() const noexcept {
    static const std::vector<double> empty;
    if (segments_.empty() || segments_.back().knots.empty()) return empty;
    return segments_.back().knots.back();
  }

  friend Trajectory plan(const std::vector<PlanSegment>&, const MotionLimits&);

 private:
  struct Seg final {
    std::vector<std::vector<double>> knots;
    std::vector<double> knot_s;
    VelocityProfile profile;  // empty for a hold segment
    double t0{0.0};
    double dur{0.0};
    double length{0.0};
    bool motion{true};
    int command_index{0};
  };

  // Quadratic Bézier corner: x (on the incoming leg) -> k (the knot) -> y (on the
  // outgoing leg), replacing the path within radius r of the corner.
  struct Blend final {
    std::size_t before{0};
    std::size_t after{0};
    double r{0.0};
    std::vector<double> x, k, y;
  };

  [[nodiscard]] std::size_t active_segment(double t) const noexcept {
    std::size_t i = 0;
    while (i + 1 < segments_.size() && t >= segments_[i + 1].t0) ++i;
    return i;
  }

  // Path coordinate of segment `s` at time t (clamped to the segment's own span).
  [[nodiscard]] static double seg_s(const Seg& s, double t) noexcept {
    if (!s.motion) return 0.0;
    const double tau = std::clamp(t - s.t0, 0.0, s.dur);
    return s.profile.sample(tau).s;
  }

  // Map a path coordinate s to a joint vector by piecewise-linear interpolation
  // across the segment's knots (s is expected within [0, length]).
  [[nodiscard]] std::vector<double> geom(const Seg& s, double sp) const {
    if (s.knots.empty()) return std::vector<double>(dof_, 0.0);
    if (s.knots.size() == 1) return s.knots.front();
    const auto& ks = s.knot_s;
    std::size_t i = 0;
    if (sp <= ks.front()) {
      i = 0;
    } else if (sp >= ks.back()) {
      i = ks.size() - 2;
    } else {
      while (i + 2 < ks.size() && sp >= ks[i + 1]) ++i;
    }
    const double span = ks[i + 1] - ks[i];
    const double u = span > 1e-12 ? std::clamp((sp - ks[i]) / span, 0.0, 1.0) : 0.0;
    std::vector<double> out(dof_, 0.0);
    for (std::size_t j = 0; j < dof_; ++j) out[j] = s.knots[i][j] + (s.knots[i + 1][j] - s.knots[i][j]) * u;
    return out;
  }

  [[nodiscard]] std::vector<double> bezier(const Blend& b, double u) const {
    const double a0 = (1 - u) * (1 - u), a1 = 2 * (1 - u) * u, a2 = u * u;
    std::vector<double> out(dof_, 0.0);
    for (std::size_t j = 0; j < dof_; ++j) out[j] = a0 * b.x[j] + a1 * b.k[j] + a2 * b.y[j];
    return out;
  }

  std::vector<Seg> segments_;
  std::vector<Blend> blends_;
  std::size_t dof_{0};
  double total_{0.0};
};

// Build a trajectory from planning segments and kinematic limits. Motion segments
// get a velocity profile (trapezoidal or S-curve); blended boundaries carry a
// matched non-zero speed so the robot does not stop at the corner, and the corner
// itself is rounded by a quadratic Bézier of radius blend_radius.
[[nodiscard]] inline Trajectory plan(const std::vector<PlanSegment>& segments,
                                     const MotionLimits& limits) {
  Trajectory traj;
  if (segments.empty()) return traj;
  for (const auto& ps : segments) {
    if (!ps.knots.empty()) {
      traj.dof_ = ps.knots.front().size();
      break;
    }
  }

  const std::size_t n = segments.size();
  std::vector<double> v_cruise(n, 0.0), a_max(n, 0.0), j_max(n, 0.0);
  for (std::size_t i = 0; i < n; ++i) {
    if (!segments[i].motion || segments[i].length() <= 1e-12) continue;
    const double v = segments[i].speed > 0.0 ? segments[i].speed : 1.0;
    v_cruise[i] = v;
    a_max[i] = limits.accel_time_s > 1e-9 ? v / limits.accel_time_s : v * 1e3;
    j_max[i] = limits.jerk_time_s > 1e-9 ? a_max[i] / limits.jerk_time_s : 0.0;
  }

  // Which boundaries are blended, and the matched boundary speed carried across
  // (capped so each side can reach it from rest within its own length).
  std::vector<double> v_start(n, 0.0), v_end(n, 0.0);
  std::vector<double> blend_r(n, 0.0);
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const auto& a = segments[i];
    const auto& b = segments[i + 1];
    const bool both_move = a.motion && b.motion && a.length() > 1e-12 && b.length() > 1e-12;
    if (!both_move || a.blend_radius <= 0.0) continue;
    double vb = std::min(v_cruise[i], v_cruise[i + 1]);
    vb = std::min(vb, std::sqrt(a_max[i] * a.length()));
    vb = std::min(vb, std::sqrt(a_max[i + 1] * b.length()));
    v_end[i] = vb;
    v_start[i + 1] = vb;
    blend_r[i] = std::min({a.blend_radius, 0.5 * a.length(), 0.5 * b.length()});
  }

  double running = 0.0;
  traj.segments_.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    const PlanSegment& ps = segments[i];
    Trajectory::Seg seg;
    seg.knots = ps.knots;
    seg.knot_s = ps.knot_s;
    seg.motion = ps.motion && ps.length() > 1e-12;
    seg.command_index = ps.command_index;
    if (seg.motion) {
      seg.length = ps.length();
      seg.profile = plan_profile(seg.length, v_cruise[i], a_max[i], j_max[i], v_start[i], v_end[i]);
      seg.dur = seg.profile.duration();
    } else {
      seg.length = 0.0;
      seg.dur = std::max(ps.hold_s, limits.min_hold_s);
    }
    seg.t0 = running;
    running += seg.dur;
    traj.segments_.push_back(std::move(seg));
  }
  traj.total_ = running;

  // Precompute the Bézier corner for each blended boundary.
  for (std::size_t i = 0; i + 1 < n; ++i) {
    if (v_end[i] <= 0.0 || blend_r[i] <= 1e-9) continue;
    const Trajectory::Seg& a = traj.segments_[i];
    const Trajectory::Seg& b = traj.segments_[i + 1];
    Trajectory::Blend bl;
    bl.before = i;
    bl.after = i + 1;
    bl.r = blend_r[i];
    bl.x = traj.geom(a, a.length - bl.r);  // enter the corner r before the knot
    bl.k = traj.geom(a, a.length);         // the shared knot (control point)
    bl.y = traj.geom(b, bl.r);             // leave the corner r after the knot
    traj.blends_.push_back(std::move(bl));
  }
  return traj;
}

}  // namespace cavr::motion
