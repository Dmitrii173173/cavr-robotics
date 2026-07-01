add_executable(cavr-record main.cpp)

find_package(Threads REQUIRED)

target_link_libraries(cavr-record
  PRIVATE
    CAVR::runtime
    CAVR::record
    CAVR::catalog
    CAVR::adapter_mock_robot
    CAVR::adapter_mock_camera
    CAVR::adapter_file_camera
    CAVR::adapter_generic_tcp_robot
    CAVR::machine
    CAVR::validation
    CAVR::core
    Threads::Threads
)

if(CAVR_ENABLE_MCAP)
  target_link_libraries(cavr-record PRIVATE CAVR::storage_mcap)
  target_compile_definitions(cavr-record PRIVATE CAVR_WITH_MCAP)
endif()

if(CAVR_ENABLE_SQLITE)
  target_compile_definitions(cavr-record PRIVATE CAVR_WITH_SQLITE)
endif()
