add_library(cavr_validation INTERFACE)
add_library(CAVR::validation ALIAS cavr_validation)

target_include_directories(cavr_validation
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cavr_validation
  INTERFACE
    CAVR::core
    CAVR::time
    CAVR::machine
)
