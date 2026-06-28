add_library(cavr_adapter_mock_camera INTERFACE)
add_library(CAVR::adapter_mock_camera ALIAS cavr_adapter_mock_camera)

target_link_libraries(cavr_adapter_mock_camera
  INTERFACE
    CAVR::adapter_sdk
)
