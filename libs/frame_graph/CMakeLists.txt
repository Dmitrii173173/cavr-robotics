add_library(cavr_frame_graph INTERFACE)
add_library(CAVR::frame_graph ALIAS cavr_frame_graph)

target_include_directories(cavr_frame_graph
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cavr_frame_graph
  INTERFACE
    CAVR::core
    CAVR::time
)
