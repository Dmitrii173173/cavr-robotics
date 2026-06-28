add_library(cavr_core INTERFACE)
add_library(CAVR::core ALIAS cavr_core)

target_include_directories(cavr_core
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
