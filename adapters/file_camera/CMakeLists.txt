add_library(cavr_adapter_file_camera INTERFACE)
add_library(CAVR::adapter_file_camera ALIAS cavr_adapter_file_camera)

target_include_directories(cavr_adapter_file_camera
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cavr_adapter_file_camera
  INTERFACE
    CAVR::adapter_sdk
)
