add_executable(cavr_core_domain_types_test
  core_domain_types_test.cpp
)

target_link_libraries(cavr_core_domain_types_test
  PRIVATE
    CAVR::core
)

add_test(NAME cavr_core_domain_types_test COMMAND cavr_core_domain_types_test)

add_executable(cavr_replay_csv_session_test
  replay_csv_session_test.cpp
)

target_link_libraries(cavr_replay_csv_session_test
  PRIVATE
    CAVR::replay
)

target_compile_definitions(cavr_replay_csv_session_test
  PRIVATE
    CAVR_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/datasets"
)

add_test(NAME cavr_replay_csv_session_test COMMAND cavr_replay_csv_session_test)

add_executable(cavr_replay_scheduler_test
  replay_scheduler_test.cpp
)

target_link_libraries(cavr_replay_scheduler_test
  PRIVATE
    CAVR::replay
)

target_compile_definitions(cavr_replay_scheduler_test
  PRIVATE
    CAVR_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/datasets"
)

add_test(NAME cavr_replay_scheduler_test COMMAND cavr_replay_scheduler_test)

add_executable(cavr_replay_interpolation_test
  replay_interpolation_test.cpp
)

target_link_libraries(cavr_replay_interpolation_test
  PRIVATE
    CAVR::replay
)

add_test(NAME cavr_replay_interpolation_test COMMAND cavr_replay_interpolation_test)

add_executable(cavr_visualization_robot_model_test
  visualization_robot_model_test.cpp
)

target_link_libraries(cavr_visualization_robot_model_test
  PRIVATE
    CAVR::visualization
    CAVR::core
)

add_test(NAME cavr_visualization_robot_model_test COMMAND cavr_visualization_robot_model_test)

add_executable(cavr_runtime_workflow_test
  runtime_workflow_test.cpp
)

target_link_libraries(cavr_runtime_workflow_test
  PRIVATE
    CAVR::runtime
    CAVR::machine
    CAVR::validation
    CAVR::adapter_mock_robot
    CAVR::core
)

add_test(NAME cavr_runtime_workflow_test COMMAND cavr_runtime_workflow_test)
