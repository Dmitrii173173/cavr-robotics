add_library(cavr_adapter_file_robot INTERFACE)
add_library(CAVR::adapter_file_robot ALIAS cavr_adapter_file_robot)

target_link_libraries(cavr_adapter_file_robot
  INTERFACE
    CAVR::adapter_sdk
)
