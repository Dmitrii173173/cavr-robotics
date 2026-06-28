add_library(twinreplay_calibration INTERFACE)
add_library(TwinReplay::calibration ALIAS twinreplay_calibration)

target_include_directories(twinreplay_calibration
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(twinreplay_calibration
  INTERFACE
    TwinReplay::core
    TwinReplay::frame_graph
)
