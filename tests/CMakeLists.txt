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
