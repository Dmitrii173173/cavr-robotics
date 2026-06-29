add_library(cavr_adapter_generic_tcp_robot INTERFACE)
add_library(CAVR::adapter_generic_tcp_robot ALIAS cavr_adapter_generic_tcp_robot)

target_link_libraries(cavr_adapter_generic_tcp_robot
  INTERFACE
    CAVR::adapter_sdk
    CAVR::transport
)
