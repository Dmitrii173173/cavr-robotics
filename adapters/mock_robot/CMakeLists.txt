add_library(cavr_adapter_mock_robot INTERFACE)
add_library(CAVR::adapter_mock_robot ALIAS cavr_adapter_mock_robot)

target_link_libraries(cavr_adapter_mock_robot
  INTERFACE
    CAVR::adapter_sdk
)
