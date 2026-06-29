add_library(cavr_visualization INTERFACE)
add_library(CAVR::visualization ALIAS cavr_visualization)

target_include_directories(cavr_visualization
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cavr_visualization
  INTERFACE
    CAVR::core
)
