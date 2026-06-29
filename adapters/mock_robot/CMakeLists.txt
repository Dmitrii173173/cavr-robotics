add_library(cavr_adapter_mock_robot INTERFACE)
add_library(CAVR::adapter_mock_robot ALIAS cavr_adapter_mock_robot)

target_include_directories(cavr_adapter_mock_robot
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cavr_adapter_mock_robot
  INTERFACE
    CAVR::adapter_sdk
    CAVR::machine
)
