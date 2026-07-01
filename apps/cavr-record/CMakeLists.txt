add_executable(cavr-record main.cpp)

target_link_libraries(cavr-record
  PRIVATE
    CAVR::runtime
    CAVR::record
    CAVR::catalog
    CAVR::adapter_mock_robot
    CAVR::adapter_mock_camera
    CAVR::adapter_file_camera
    CAVR::machine
    CAVR::validation
    CAVR::core
)

if(CAVR_ENABLE_MCAP)
  target_link_libraries(cavr-record PRIVATE CAVR::storage_mcap)
  target_compile_definitions(cavr-record PRIVATE CAVR_WITH_MCAP)
endif()

if(CAVR_ENABLE_SQLITE)
  target_compile_definitions(cavr-record PRIVATE CAVR_WITH_SQLITE)
endif()
