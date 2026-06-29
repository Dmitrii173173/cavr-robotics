add_library(cavr_calibration INTERFACE)
add_library(CAVR::calibration ALIAS cavr_calibration)

target_include_directories(cavr_calibration
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cavr_calibration
  INTERFACE
    CAVR::core
    CAVR::frame_graph
)
