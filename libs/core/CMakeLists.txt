add_library(twinreplay_core INTERFACE)
add_library(TwinReplay::core ALIAS twinreplay_core)

target_include_directories(twinreplay_core
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
