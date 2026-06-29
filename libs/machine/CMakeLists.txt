add_library(cavr_machine INTERFACE)
add_library(CAVR::machine ALIAS cavr_machine)

target_include_directories(cavr_machine
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cavr_machine
  INTERFACE
    CAVR::core
)
