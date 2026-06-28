add_library(cavr_time INTERFACE)
add_library(CAVR::time ALIAS cavr_time)

target_include_directories(cavr_time
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cavr_time
  INTERFACE
    CAVR::core
)
