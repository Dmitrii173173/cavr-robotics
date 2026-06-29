#pragma once

// Controlled vocabulary shared across the machine-configuration, adapter and
// runtime layers. Every enum has stable string forms so profiles and session
// logs round-trip through JSON without binary coupling.

#include <array>
#include <string>
#include <string_view>
#include <utility>

namespace cavr::machine {

enum class JointType { Revolute, Prismatic };

// Standard industrial frame roles.
enum class FrameKind { World, Base, Flange, Tool, User, Camera, Object };

enum class IoKind { Digital, Analog };
enum class IoDirection { Input, Output };

// Telemetry channel payloads the application understands.
enum class ChannelKind {
  JointPosition, CartesianPose, Speed, ProgramState, IoState,
  Error, Event, CameraImage, DepthImage, PointCloud, Custom
};

// Industrial motion primitives.
enum class MotionKind { MoveJ, MoveL, MoveC, Wait, ToolOn, ToolOff };

// Live controller states.
enum class ProgramState { Idle, Loaded, Running, Paused, Holding, Completed, Aborted };
enum class ServoState { Off, On, Error };
enum class Severity { Info, Warning, Error, Critical };

// Controller-reported event categories.
enum class EventKind {
  SessionStarted, ProgramStarted, StepStarted, StepCompleted,
  IoChanged, Warning, Error, EmergencyStop, ProgramCompleted, SessionStopped
};

namespace detail {

template <typename E, std::size_t N>
[[nodiscard]] inline std::string enum_to_string(E value, const std::array<std::pair<E, std::string_view>, N>& table) {
  for (const auto& [key, name] : table) {
    if (key == value) return std::string(name);
  }
  return "unknown";
}

template <typename E, std::size_t N>
[[nodiscard]] inline E enum_from_string(std::string_view text, const std::array<std::pair<E, std::string_view>, N>& table, E fallback) {
  for (const auto& [key, name] : table) {
    if (name == text) return key;
  }
  return fallback;
}

inline constexpr std::array<std::pair<JointType, std::string_view>, 2> kJointType{{
    {JointType::Revolute, "revolute"}, {JointType::Prismatic, "prismatic"}}};

inline constexpr std::array<std::pair<FrameKind, std::string_view>, 7> kFrameKind{{
    {FrameKind::World, "world"}, {FrameKind::Base, "base"}, {FrameKind::Flange, "flange"},
    {FrameKind::Tool, "tool"}, {FrameKind::User, "user"}, {FrameKind::Camera, "camera"},
    {FrameKind::Object, "object"}}};

inline constexpr std::array<std::pair<IoKind, std::string_view>, 2> kIoKind{{
    {IoKind::Digital, "digital"}, {IoKind::Analog, "analog"}}};

inline constexpr std::array<std::pair<IoDirection, std::string_view>, 2> kIoDirection{{
    {IoDirection::Input, "input"}, {IoDirection::Output, "output"}}};

inline constexpr std::array<std::pair<ChannelKind, std::string_view>, 11> kChannelKind{{
    {ChannelKind::JointPosition, "joint_position"}, {ChannelKind::CartesianPose, "cartesian_pose"},
    {ChannelKind::Speed, "speed"}, {ChannelKind::ProgramState, "program_state"},
    {ChannelKind::IoState, "io_state"}, {ChannelKind::Error, "error"}, {ChannelKind::Event, "event"},
    {ChannelKind::CameraImage, "camera_image"}, {ChannelKind::DepthImage, "depth_image"},
    {ChannelKind::PointCloud, "point_cloud"}, {ChannelKind::Custom, "custom"}}};

inline constexpr std::array<std::pair<MotionKind, std::string_view>, 6> kMotionKind{{
    {MotionKind::MoveJ, "movej"}, {MotionKind::MoveL, "movel"}, {MotionKind::MoveC, "movec"},
    {MotionKind::Wait, "wait"}, {MotionKind::ToolOn, "tool_on"}, {MotionKind::ToolOff, "tool_off"}}};

inline constexpr std::array<std::pair<ProgramState, std::string_view>, 7> kProgramState{{
    {ProgramState::Idle, "idle"}, {ProgramState::Loaded, "loaded"}, {ProgramState::Running, "running"},
    {ProgramState::Paused, "paused"}, {ProgramState::Holding, "holding"},
    {ProgramState::Completed, "completed"}, {ProgramState::Aborted, "aborted"}}};

inline constexpr std::array<std::pair<ServoState, std::string_view>, 3> kServoState{{
    {ServoState::Off, "off"}, {ServoState::On, "on"}, {ServoState::Error, "error"}}};

inline constexpr std::array<std::pair<Severity, std::string_view>, 4> kSeverity{{
    {Severity::Info, "info"}, {Severity::Warning, "warning"},
    {Severity::Error, "error"}, {Severity::Critical, "critical"}}};

inline constexpr std::array<std::pair<EventKind, std::string_view>, 10> kEventKind{{
    {EventKind::SessionStarted, "session_started"}, {EventKind::ProgramStarted, "program_started"},
    {EventKind::StepStarted, "step_started"}, {EventKind::StepCompleted, "step_completed"},
    {EventKind::IoChanged, "io_changed"}, {EventKind::Warning, "warning"}, {EventKind::Error, "error"},
    {EventKind::EmergencyStop, "emergency_stop"}, {EventKind::ProgramCompleted, "program_completed"},
    {EventKind::SessionStopped, "session_stopped"}}};

}  // namespace detail

[[nodiscard]] inline std::string to_string(JointType v) { return detail::enum_to_string(v, detail::kJointType); }
[[nodiscard]] inline std::string to_string(FrameKind v) { return detail::enum_to_string(v, detail::kFrameKind); }
[[nodiscard]] inline std::string to_string(IoKind v) { return detail::enum_to_string(v, detail::kIoKind); }
[[nodiscard]] inline std::string to_string(IoDirection v) { return detail::enum_to_string(v, detail::kIoDirection); }
[[nodiscard]] inline std::string to_string(ChannelKind v) { return detail::enum_to_string(v, detail::kChannelKind); }
[[nodiscard]] inline std::string to_string(MotionKind v) { return detail::enum_to_string(v, detail::kMotionKind); }
[[nodiscard]] inline std::string to_string(ProgramState v) { return detail::enum_to_string(v, detail::kProgramState); }
[[nodiscard]] inline std::string to_string(ServoState v) { return detail::enum_to_string(v, detail::kServoState); }
[[nodiscard]] inline std::string to_string(Severity v) { return detail::enum_to_string(v, detail::kSeverity); }
[[nodiscard]] inline std::string to_string(EventKind v) { return detail::enum_to_string(v, detail::kEventKind); }

[[nodiscard]] inline JointType joint_type_from_string(std::string_view s) { return detail::enum_from_string(s, detail::kJointType, JointType::Revolute); }
[[nodiscard]] inline FrameKind frame_kind_from_string(std::string_view s) { return detail::enum_from_string(s, detail::kFrameKind, FrameKind::User); }
[[nodiscard]] inline IoKind io_kind_from_string(std::string_view s) { return detail::enum_from_string(s, detail::kIoKind, IoKind::Digital); }
[[nodiscard]] inline IoDirection io_direction_from_string(std::string_view s) { return detail::enum_from_string(s, detail::kIoDirection, IoDirection::Input); }
[[nodiscard]] inline ChannelKind channel_kind_from_string(std::string_view s) { return detail::enum_from_string(s, detail::kChannelKind, ChannelKind::Custom); }
[[nodiscard]] inline MotionKind motion_kind_from_string(std::string_view s) { return detail::enum_from_string(s, detail::kMotionKind, MotionKind::MoveJ); }
[[nodiscard]] inline ProgramState program_state_from_string(std::string_view s) { return detail::enum_from_string(s, detail::kProgramState, ProgramState::Idle); }
[[nodiscard]] inline ServoState servo_state_from_string(std::string_view s) { return detail::enum_from_string(s, detail::kServoState, ServoState::Off); }
[[nodiscard]] inline Severity severity_from_string(std::string_view s) { return detail::enum_from_string(s, detail::kSeverity, Severity::Info); }
[[nodiscard]] inline EventKind event_kind_from_string(std::string_view s) { return detail::enum_from_string(s, detail::kEventKind, EventKind::Warning); }

}  // namespace cavr::machine
