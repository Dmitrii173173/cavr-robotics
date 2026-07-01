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

add_executable(cavr_record_recording_test
  record_recording_test.cpp
)

target_link_libraries(cavr_record_recording_test
  PRIVATE
    CAVR::record
)

add_test(NAME cavr_record_recording_test COMMAND cavr_record_recording_test)

if(CAVR_ENABLE_MCAP)
  add_executable(cavr_storage_mcap_recording_test
    storage_mcap_recording_test.cpp
  )

  target_link_libraries(cavr_storage_mcap_recording_test
    PRIVATE
      CAVR::storage_mcap
      CAVR::record
  )

  add_test(NAME cavr_storage_mcap_recording_test COMMAND cavr_storage_mcap_recording_test)
endif()

add_executable(cavr_runtime_session_recording_test
  runtime_session_recording_test.cpp
)

target_link_libraries(cavr_runtime_session_recording_test
  PRIVATE
    CAVR::runtime
    CAVR::record
    CAVR::adapter_mock_robot
    CAVR::machine
    CAVR::validation
    CAVR::core
)

# Also exercise the bridge through the MCAP backend when it is built.
if(CAVR_ENABLE_MCAP)
  target_link_libraries(cavr_runtime_session_recording_test PRIVATE CAVR::storage_mcap)
  target_compile_definitions(cavr_runtime_session_recording_test PRIVATE CAVR_WITH_MCAP)
endif()

add_test(NAME cavr_runtime_session_recording_test COMMAND cavr_runtime_session_recording_test)

add_executable(cavr_runtime_session_recorder_test
  runtime_session_recorder_test.cpp
)

target_link_libraries(cavr_runtime_session_recorder_test
  PRIVATE
    CAVR::runtime
    CAVR::record
    CAVR::adapter_mock_robot
    CAVR::machine
    CAVR::validation
    CAVR::core
)

if(CAVR_ENABLE_MCAP)
  target_link_libraries(cavr_runtime_session_recorder_test PRIVATE CAVR::storage_mcap)
  target_compile_definitions(cavr_runtime_session_recorder_test PRIVATE CAVR_WITH_MCAP)
endif()

add_test(NAME cavr_runtime_session_recorder_test COMMAND cavr_runtime_session_recorder_test)

add_executable(cavr_runtime_camera_recording_test
  runtime_camera_recording_test.cpp
)

target_link_libraries(cavr_runtime_camera_recording_test
  PRIVATE
    CAVR::runtime
    CAVR::record
    CAVR::adapter_mock_robot
    CAVR::adapter_mock_camera
    CAVR::machine
    CAVR::validation
    CAVR::core
)

if(CAVR_ENABLE_MCAP)
  target_link_libraries(cavr_runtime_camera_recording_test PRIVATE CAVR::storage_mcap)
  target_compile_definitions(cavr_runtime_camera_recording_test PRIVATE CAVR_WITH_MCAP)
endif()

add_test(NAME cavr_runtime_camera_recording_test COMMAND cavr_runtime_camera_recording_test)

add_executable(cavr_catalog_test
  catalog_test.cpp
)

target_link_libraries(cavr_catalog_test
  PRIVATE
    CAVR::catalog
)

if(CAVR_ENABLE_SQLITE)
  target_compile_definitions(cavr_catalog_test PRIVATE CAVR_WITH_SQLITE)
  # the test fabricates a future-version database directly; sqlite symbols come
  # from CAVR::catalog, this only adds the vendored header to its include path.
  target_include_directories(cavr_catalog_test PRIVATE ${CMAKE_SOURCE_DIR}/third_party/sqlite)
endif()

add_test(NAME cavr_catalog_test COMMAND cavr_catalog_test)

add_executable(cavr_runtime_catalog_index_test
  runtime_catalog_index_test.cpp
)

target_link_libraries(cavr_runtime_catalog_index_test
  PRIVATE
    CAVR::runtime
    CAVR::record
    CAVR::catalog
    CAVR::adapter_mock_robot
    CAVR::adapter_mock_camera
    CAVR::machine
    CAVR::validation
    CAVR::core
)

if(CAVR_ENABLE_MCAP)
  target_link_libraries(cavr_runtime_catalog_index_test PRIVATE CAVR::storage_mcap)
  target_compile_definitions(cavr_runtime_catalog_index_test PRIVATE CAVR_WITH_MCAP)
endif()

if(CAVR_ENABLE_SQLITE)
  target_compile_definitions(cavr_runtime_catalog_index_test PRIVATE CAVR_WITH_SQLITE)
endif()

add_test(NAME cavr_runtime_catalog_index_test COMMAND cavr_runtime_catalog_index_test)

add_executable(cavr_file_camera_test
  file_camera_test.cpp
)

target_link_libraries(cavr_file_camera_test
  PRIVATE
    CAVR::adapter_file_camera
    CAVR::core
)

add_test(NAME cavr_file_camera_test COMMAND cavr_file_camera_test)
