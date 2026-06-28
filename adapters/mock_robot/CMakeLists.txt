add_library(twinreplay_adapter_mock_robot INTERFACE)
add_library(TwinReplay::adapter_mock_robot ALIAS twinreplay_adapter_mock_robot)

target_link_libraries(twinreplay_adapter_mock_robot
  INTERFACE
    TwinReplay::adapter_sdk
)
