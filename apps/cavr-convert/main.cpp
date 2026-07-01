// cavr-convert: converts a recording between storage backends — the MCAP
// authoritative store and the JSON reference backend — by loading it into the
// neutral record::Recording model and writing it back through the other backend.
// The message stream is preserved exactly (channels, payloads, log times); only
// the container format changes.

#include <cavr/record/copy.hpp>
#include <cavr/record/json_recording.hpp>

#ifdef CAVR_WITH_MCAP
#include <cavr/storage_mcap/mcap_recording.hpp>
#endif

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace {

namespace record = cavr::record;

void print_usage() {
  std::cout <<
      "Usage: cavr-convert <input> <output>\n\n"
      "Converts a recording between backends. The extension of each path selects\n"
      "the backend: .mcap (MCAP) or .json (reference backend).\n\n"
      "  cavr-convert session.mcap session.json   # MCAP  -> JSON\n"
      "  cavr-convert session.json session.mcap   # JSON  -> MCAP\n";
}

// Loads a recording from either backend, chosen by extension.
[[nodiscard]] bool load(const std::filesystem::path& path, record::Recording& out) {
  const std::string ext = path.extension().string();
  if (ext == ".mcap") {
#ifdef CAVR_WITH_MCAP
    auto loaded = cavr::storage_mcap::load_recording(path);
    if (!loaded.status) {
      std::cerr << "error: failed to read " << path.string() << ": " << loaded.status.error << '\n';
      return false;
    }
    out = std::move(loaded.recording);
    return true;
#else
    std::cerr << "error: this build was configured with CAVR_ENABLE_MCAP=OFF; cannot read .mcap files\n";
    return false;
#endif
  }
  if (ext == ".json") {
    auto loaded = record::load_recording(path);
    if (!loaded.status) {
      std::cerr << "error: failed to read " << path.string() << ": " << loaded.status.error << '\n';
      return false;
    }
    out = std::move(loaded.recording);
    return true;
  }
  std::cerr << "error: unsupported input extension '" << ext << "' (expected .mcap or .json)\n";
  return false;
}

// Makes a writer for either backend, chosen by extension.
[[nodiscard]] std::unique_ptr<record::RecordingWriter> make_writer(const std::filesystem::path& path) {
  const std::string ext = path.extension().string();
  if (ext == ".mcap") {
#ifdef CAVR_WITH_MCAP
    return std::make_unique<cavr::storage_mcap::McapRecordingWriter>(path);
#else
    std::cerr << "error: this build was configured with CAVR_ENABLE_MCAP=OFF; cannot write .mcap files\n";
    return nullptr;
#endif
  }
  if (ext == ".json") {
    return std::make_unique<record::JsonRecordingWriter>(path);
  }
  std::cerr << "error: unsupported output extension '" << ext << "' (expected .mcap or .json)\n";
  return nullptr;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
    print_usage();
    return 0;
  }
  if (argc != 3) {
    print_usage();
    return 2;
  }

  const std::filesystem::path input = argv[1];
  const std::filesystem::path output = argv[2];

  record::Recording recording;
  if (!load(input, recording)) return 1;

  std::unique_ptr<record::RecordingWriter> writer = make_writer(output);
  if (!writer) return 1;

  if (const record::RecordStatus status = record::write_recording(recording, *writer); !status) {
    std::cerr << "error: failed to write " << output.string() << ": " << status.error << '\n';
    return 1;
  }

  std::cout << "Converted " << input.string() << " -> " << output.string() << " ("
            << recording.channels.size() << " channels, " << recording.messages.size() << " messages).\n";
  return 0;
}
