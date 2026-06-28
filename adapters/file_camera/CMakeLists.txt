add_library(twinreplay_adapter_file_camera INTERFACE)
add_library(TwinReplay::adapter_file_camera ALIAS twinreplay_adapter_file_camera)

target_link_libraries(twinreplay_adapter_file_camera
  INTERFACE
    TwinReplay::adapter_sdk
)
