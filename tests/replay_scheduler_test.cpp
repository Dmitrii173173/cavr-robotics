#include <cavr/replay/scheduler.hpp>
#include <cavr/replay/session_loader.hpp>

#include <filesystem>
#include <iostream>
#include <string_view>

#ifndef CAVR_TEST_DATA_DIR
#error "CAVR_TEST_DATA_DIR must be defined"
#endif

namespace {

int failures = 0;

void check(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    ++failures;
  }
}

cavr::replay::LoadedSession load_demo_session() {
  const auto manifest_path = std::filesystem::path(CAVR_TEST_DATA_DIR) / "demo_csv" / "session.json";
  const auto result = cavr::replay::load_session_manifest(manifest_path);
  if (!result.ok()) {
    for (const auto& error : result.errors) {
      std::cerr << "load error: " << error << '\n';
    }
    ++failures;
  }
  return result.session;
}

}  // namespace

int main() {
  const auto session = load_demo_session();
  const auto timeline = cavr::replay::build_timeline(session);

  check(timeline.size() == 3, "timeline contains three events");
  check(!timeline.empty(), "timeline is not empty");
  check(timeline.event_at(0)->timestamp.nanoseconds() == 0, "first event timestamp");
  check(timeline.event_at(1)->timestamp.nanoseconds() == 100'000'000, "second event timestamp");
  check(timeline.event_at(2)->timestamp.nanoseconds() == 200'000'000, "third event timestamp");

  cavr::replay::ReplayCursor cursor(timeline);
  const auto* first = cursor.step_next();
  const auto* second = cursor.step_next();
  const auto* third = cursor.step_next();
  const auto* done = cursor.step_next();

  check(first != nullptr && first->sample.image_path.filename() == "frame_0000.png", "step first event");
  check(second != nullptr && second->sample.image_path.filename() == "frame_0001.png", "step second event");
  check(third != nullptr && third->sample.image_path.filename() == "frame_0002.png", "step third event");
  check(done == nullptr, "step past end returns null");
  check(cursor.done(), "cursor is done after final event");

  cursor.reset();
  check(cursor.index() == 0, "reset returns cursor to start");

  const auto* seek_100ms = cursor.seek_to(cavr::core::Timestamp::from_nanoseconds(100'000'000));
  check(seek_100ms != nullptr, "seek returns an event");
  check(seek_100ms != nullptr && seek_100ms->sample.image_path.filename() == "frame_0001.png", "seek to exact timestamp");

  const auto* seek_between = cursor.seek_to(cavr::core::Timestamp::from_nanoseconds(150'000'000));
  check(seek_between != nullptr && seek_between->sample.image_path.filename() == "frame_0002.png",
        "seek to intermediate timestamp returns next event");

  const auto* seek_after_end = cursor.seek_to(cavr::core::Timestamp::from_nanoseconds(300'000'000));
  check(seek_after_end == nullptr, "seek after end returns null");
  check(cursor.done(), "cursor is done after seek past end");

  const cavr::replay::ReplayTimeline empty_timeline;
  cavr::replay::ReplayCursor empty_cursor(empty_timeline);
  check(empty_timeline.empty(), "empty timeline reports empty");
  check(empty_cursor.step_next() == nullptr, "empty cursor has no next event");

  return failures == 0 ? 0 : 1;
}
