add_library(twinreplay_adapter_mock_camera INTERFACE)
add_library(TwinReplay::adapter_mock_camera ALIAS twinreplay_adapter_mock_camera)

target_link_libraries(twinreplay_adapter_mock_camera
  INTERFACE
    TwinReplay::adapter_sdk
)
