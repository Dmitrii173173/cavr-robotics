add_library(twinreplay_fault_injection INTERFACE)
add_library(TwinReplay::fault_injection ALIAS twinreplay_fault_injection)

target_include_directories(twinreplay_fault_injection
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(twinreplay_fault_injection
  INTERFACE
    TwinReplay::core
    TwinReplay::time
)
