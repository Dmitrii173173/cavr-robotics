// cavr-record: runs the demo GP25 welding workflow against the mock robot and
// camera adapters, streaming a synchronized robot + vision session live to a
// recording (MCAP when built, JSON otherwise) exactly as CAVR Studio's Monitor
// phase would, and optionally indexes the finished recording into the catalog.
//
// This is the first end-to-end exercise of the record -> storage -> catalog
// backend from the command line, rather than only from unit tests.

#include <cavr/adapter_sdk/camera_adapter.hpp>
#include <cavr/adapters/file_camera/file_camera_adapter.hpp>
#include <cavr/adapters/mock_camera/mock_camera.hpp>
#include <cavr/adapters/mock_robot/mock_controller.hpp>
#include <cavr/catalog/in_memory_catalog.hpp>
#include <cavr/record/json_recording.hpp>
#include <cavr/runtime/catalog_index.hpp>
#include <cavr/runtime/demo_plan.hpp>
#include <cavr/runtime/session_manager.hpp>
#include <cavr/runtime/session_recorder.hpp>

#ifdef CAVR_WITH_MCAP
#include <cavr/storage_mcap/mcap_recording.hpp>
#endif
#ifdef CAVR_WITH_SQLITE
#include <cavr/catalog/sqlite_catalog.hpp>
#endif

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace {

namespace runtime = cavr::runtime;
namespace record = cavr::record;
namespace catalog = cavr::catalog;
namespace mock_robot = cavr::adapters::mock_robot;
namespace mock_camera = cavr::adapters::mock_camera;
namespace file_camera = cavr::adapters::file_camera;

constexpr std::int64_t kTickNs = 20'000'000;  // 50 Hz simulated clock, matches the Studio demo

struct Options {
  std::filesystem::path out;
  std::string session_id;
  std::string catalog_path;
  std::string frames_dir;
  double fps{30.0};
  int max_ticks{5000};
};

[[nodiscard]] std::string default_out_path() {
#ifdef CAVR_WITH_MCAP
  return "recording.mcap";
#else
  return "recording.json";
#endif
}

[[nodiscard]] std::string default_session_id() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now).count();
  return "cavr-record-" + std::to_string(seconds);
}

void print_usage() {
  std::cout <<
      "Usage: cavr-record [--out <path>] [--session-id <id>] [--catalog <path>] "
      "[--frames-dir <dir>] [--fps <n>] [--ticks <n>]\n\n"
      "Records the demo GP25 welding workflow (mock robot + camera) as a\n"
      "synchronized robot + vision session.\n\n"
      "  --out <path>        Recording file to write. Extension selects the backend:\n"
      "                      .mcap (MCAP) or .json (reference backend). Default: "
      << default_out_path() << "\n"
      "  --session-id <id>   Session identifier. Default: a timestamped id.\n"
      "  --catalog <path>    Also index the finished recording into a catalog at\n"
      "                      this path (SQLite when built, else in-memory only).\n"
      "  --frames-dir <dir>  Replay .pgm/.ppm image files from this directory as the\n"
      "                      camera stream (FileCameraAdapter) instead of the\n"
      "                      synthetic MockCamera pattern.\n"
      "  --fps <n>           Playback rate for --frames-dir. Default: 30.\n"
      "  --ticks <n>         Safety cap on simulated ticks. Default: 5000.\n";
}

// Returns std::nullopt-like failure via non-zero optional exit code, or -1 to continue.
int parse_args(int argc, char** argv, Options& opts) {
  opts.out = default_out_path();
  opts.session_id = default_session_id();

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "error: " << arg << " requires a value\n";
        return {};
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      print_usage();
      return 0;
    } else if (arg == "--out") {
      opts.out = next();
    } else if (arg == "--session-id") {
      opts.session_id = next();
    } else if (arg == "--catalog") {
      opts.catalog_path = next();
    } else if (arg == "--frames-dir") {
      opts.frames_dir = next();
    } else if (arg == "--fps") {
      opts.fps = std::stod(next());
    } else if (arg == "--ticks") {
      opts.max_ticks = std::stoi(next());
    } else {
      std::cerr << "error: unrecognized argument '" << arg << "'\n";
      print_usage();
      return 2;
    }
  }
  return -1;
}

}  // namespace

int main(int argc, char** argv) {
  Options opts;
  if (const int early_exit = parse_args(argc, argv, opts); early_exit >= 0) {
    return early_exit;
  }

  const std::string ext = opts.out.extension().string();
  bool use_mcap = false;
  if (ext == ".mcap") {
#ifdef CAVR_WITH_MCAP
    use_mcap = true;
#else
    std::cerr << "error: this build was configured with CAVR_ENABLE_MCAP=OFF; "
                 "use a .json --out path instead\n";
    return 2;
#endif
  } else if (ext == ".json") {
    use_mcap = false;
  } else {
    std::cerr << "error: unsupported output extension '" << ext << "' (expected .mcap or .json)\n";
    return 2;
  }

  std::unique_ptr<record::RecordingWriter> writer;
#ifdef CAVR_WITH_MCAP
  if (use_mcap) {
    writer = std::make_unique<cavr::storage_mcap::McapRecordingWriter>(opts.out, /*streaming=*/true);
  }
#endif
  if (!writer) {
    writer = std::make_unique<record::JsonRecordingWriter>(opts.out);
  }

  mock_robot::MockController controller;

  std::unique_ptr<cavr::adapter_sdk::CameraAdapter> camera;
  if (opts.frames_dir.empty()) {
    camera = std::make_unique<mock_camera::MockCamera>(8, 8, "weld_cam");
  } else {
    auto file_cam = std::make_unique<file_camera::FileCameraAdapter>(
        file_camera::FileCameraAdapter::from_directory(opts.frames_dir, "file_cam", opts.fps));
    if (file_cam->frame_count() == 0) {
      std::cerr << "warning: no .pgm/.ppm files found under " << opts.frames_dir << '\n';
    }
    camera = std::move(file_cam);
  }

  runtime::SessionManager manager;
  runtime::SessionRecorder recorder(*writer);
  manager.attach_recorder(recorder);
  manager.attach_camera(*camera);

  static_cast<void>(manager.connect(controller, {"mock", "mock"}));
  static_cast<void>(manager.discover_profile());
  manager.set_plan(runtime::make_demo_plan());
  const auto validation_report = manager.validate();
  if (!validation_report.ok()) {
    std::cerr << "error: demo plan failed validation against the discovered profile:\n";
    for (const auto& issue : validation_report.issues) {
      std::cerr << "  - " << issue.message << '\n';
    }
    return 1;
  }
  if (!manager.execute(opts.session_id)) {
    std::cerr << "error: failed to start session execution\n";
    return 1;
  }

  std::cout << "Recording session '" << opts.session_id << "' to " << opts.out.string() << " ("
            << (use_mcap ? "mcap" : "json") << ")...\n";

  std::int64_t now_ns = 1'000'000'000;
  int ticks = 0;
  while (manager.phase() == runtime::SessionPhase::Executing && ticks < opts.max_ticks) {
    manager.tick(cavr::core::Timestamp::from_nanoseconds(now_ns));
    now_ns += kTickNs;
    ++ticks;
  }
  if (manager.phase() != runtime::SessionPhase::Completed) {
    std::cerr << "warning: session did not complete within " << opts.max_ticks << " ticks\n";
  }

  if (const record::RecordStatus status = recorder.finish(manager.log()); !status) {
    std::cerr << "error: failed to finalize recording: " << status.error << '\n';
    return 1;
  }

  std::cout << "Wrote " << recorder.frames_written() << " telemetry frames, "
            << recorder.camera_frames_written() << " camera frames, " << recorder.events_written()
            << " events (" << recorder.errors() << " errors).\n";

  if (opts.catalog_path.empty()) {
    return 0;
  }

  record::Recording loaded_recording;
  record::RecordStatus load_status;
#ifdef CAVR_WITH_MCAP
  if (use_mcap) {
    auto loaded = cavr::storage_mcap::load_recording(opts.out);
    load_status = loaded.status;
    loaded_recording = std::move(loaded.recording);
  } else
#endif
  {
    auto loaded = record::load_recording(opts.out);
    load_status = loaded.status;
    loaded_recording = std::move(loaded.recording);
  }
  if (!load_status) {
    std::cerr << "error: failed to reload recording for cataloging: " << load_status.error << '\n';
    return 1;
  }

  // Both concrete readers are plain wrappers around the neutral Recording model, so
  // the reference reader is enough here regardless of which backend wrote the file.
  const record::JsonRecordingReader reader(std::move(loaded_recording));

  const auto fingerprint = catalog::file_fingerprint(opts.out.string());
  const catalog::CatalogSession entry =
      runtime::index_recording(reader, opts.out.string(), fingerprint.size, fingerprint.hash);
  const catalog::ValidationSummary summary =
      runtime::to_validation_summary(validation_report, manager.log().started.nanoseconds());

  std::unique_ptr<catalog::Catalog> cat;
#ifdef CAVR_WITH_SQLITE
  cat = std::make_unique<catalog::SqliteCatalog>(catalog::CatalogOpenOptions{opts.catalog_path, true});
#else
  cat = std::make_unique<catalog::InMemoryCatalog>();
  std::cout << "note: built with CAVR_ENABLE_SQLITE=OFF; catalog entry is indexed in-memory only "
               "and will not persist\n";
#endif

  if (const catalog::CatalogStatus status = cat->initialize(); !status) {
    std::cerr << "error: failed to open catalog: " << status.error << '\n';
    return 1;
  }
  if (const catalog::CatalogStatus status = cat->upsert_session(entry); !status) {
    std::cerr << "error: failed to index session into catalog: " << status.error << '\n';
    return 1;
  }
  static_cast<void>(cat->add_validation_summary(entry.id, summary));

  std::cout << "Indexed session '" << entry.id << "' (" << entry.robot_model << ", "
            << entry.camera_model << ") into catalog " << opts.catalog_path << '\n';
  return 0;
}
