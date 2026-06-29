add_library(cavr_replay INTERFACE)
add_library(CAVR::replay ALIAS cavr_replay)

target_include_directories(cavr_replay
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cavr_replay
  INTERFACE
    CAVR::core
    CAVR::time
    CAVR::adapter_sdk
)
