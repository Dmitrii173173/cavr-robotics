add_executable(cavr-robotd main.cpp)

find_package(Threads REQUIRED)

target_link_libraries(cavr-robotd
  PRIVATE
    CAVR::adapter_generic_tcp_robot
    CAVR::adapter_mock_robot
    CAVR::runtime
    CAVR::machine
    CAVR::core
    Threads::Threads
)
