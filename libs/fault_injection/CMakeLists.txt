add_library(cavr_fault_injection INTERFACE)
add_library(CAVR::fault_injection ALIAS cavr_fault_injection)

target_include_directories(cavr_fault_injection
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cavr_fault_injection
  INTERFACE
    CAVR::core
    CAVR::time
)
