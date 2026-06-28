add_library(twinreplay_adapter_generic_tcp_robot INTERFACE)
add_library(TwinReplay::adapter_generic_tcp_robot ALIAS twinreplay_adapter_generic_tcp_robot)

target_link_libraries(twinreplay_adapter_generic_tcp_robot
  INTERFACE
    TwinReplay::adapter_sdk
    TwinReplay::transport
)
