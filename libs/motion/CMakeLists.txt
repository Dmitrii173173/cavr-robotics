add_library(cavr_motion INTERFACE)
add_library(CAVR::motion ALIAS cavr_motion)

target_include_directories(cavr_motion
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cavr_motion
  INTERFACE
    CAVR::core
    CAVR::machine
)
