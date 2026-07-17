add_library(cavr_sim INTERFACE)
add_library(CAVR::sim ALIAS cavr_sim)

target_include_directories(cavr_sim
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cavr_sim
  INTERFACE
    CAVR::core
    CAVR::time
    CAVR::machine
    CAVR::motion
    CAVR::fault_injection
    CAVR::adapter_sdk
)
