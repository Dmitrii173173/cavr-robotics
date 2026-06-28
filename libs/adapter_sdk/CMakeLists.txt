add_library(twinreplay_adapter_sdk INTERFACE)
add_library(TwinReplay::adapter_sdk ALIAS twinreplay_adapter_sdk)

target_include_directories(twinreplay_adapter_sdk
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(twinreplay_adapter_sdk
  INTERFACE
    TwinReplay::core
)
