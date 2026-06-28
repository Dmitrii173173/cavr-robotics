add_library(twinreplay_adapter_opencv_camera INTERFACE)
add_library(TwinReplay::adapter_opencv_camera ALIAS twinreplay_adapter_opencv_camera)

target_link_libraries(twinreplay_adapter_opencv_camera
  INTERFACE
    TwinReplay::adapter_sdk
)
