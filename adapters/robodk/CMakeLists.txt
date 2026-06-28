add_library(cavr_adapter_robodk INTERFACE)
add_library(CAVR::adapter_robodk ALIAS cavr_adapter_robodk)

target_link_libraries(cavr_adapter_robodk
  INTERFACE
    CAVR::adapter_sdk
)
