add_library(twinreplay_adapter_file_robot INTERFACE)
add_library(TwinReplay::adapter_file_robot ALIAS twinreplay_adapter_file_robot)

target_link_libraries(twinreplay_adapter_file_robot
  INTERFACE
    TwinReplay::adapter_sdk
)
