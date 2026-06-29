add_library(cavr_adapter_file_camera INTERFACE)
add_library(CAVR::adapter_file_camera ALIAS cavr_adapter_file_camera)

target_link_libraries(cavr_adapter_file_camera
  INTERFACE
    CAVR::adapter_sdk
)
