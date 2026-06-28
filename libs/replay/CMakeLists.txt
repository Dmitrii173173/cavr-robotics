add_library(twinreplay_replay INTERFACE)
add_library(TwinReplay::replay ALIAS twinreplay_replay)

target_include_directories(twinreplay_replay
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(twinreplay_replay
  INTERFACE
    TwinReplay::core
    TwinReplay::time
    TwinReplay::adapter_sdk
)
