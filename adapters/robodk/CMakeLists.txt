add_library(twinreplay_adapter_robodk INTERFACE)
add_library(TwinReplay::adapter_robodk ALIAS twinreplay_adapter_robodk)

target_link_libraries(twinreplay_adapter_robodk
  INTERFACE
    TwinReplay::adapter_sdk
)
