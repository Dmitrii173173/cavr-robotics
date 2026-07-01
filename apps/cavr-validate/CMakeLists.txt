add_executable(cavr-validate main.cpp)

target_link_libraries(cavr-validate
  PRIVATE
    CAVR::runtime
    CAVR::validation
    CAVR::machine
    CAVR::adapter_mock_robot
    CAVR::core
)
