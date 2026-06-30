add_library(cavr_adapter_mock_camera INTERFACE)
add_library(CAVR::adapter_mock_camera ALIAS cavr_adapter_mock_camera)

target_include_directories(cavr_adapter_mock_camera
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cavr_adapter_mock_camera
  INTERFACE
    CAVR::adapter_sdk
)
