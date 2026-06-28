add_library(twinreplay_frame_graph INTERFACE)
add_library(TwinReplay::frame_graph ALIAS twinreplay_frame_graph)

target_include_directories(twinreplay_frame_graph
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(twinreplay_frame_graph
  INTERFACE
    TwinReplay::core
    TwinReplay::time
)
