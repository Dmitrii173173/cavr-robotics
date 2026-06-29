add_library(cavr_adapter_opencv_camera INTERFACE)
add_library(CAVR::adapter_opencv_camera ALIAS cavr_adapter_opencv_camera)

target_link_libraries(cavr_adapter_opencv_camera
  INTERFACE
    CAVR::adapter_sdk
)
