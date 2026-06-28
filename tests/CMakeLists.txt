add_executable(cavr_core_domain_types_test
  core_domain_types_test.cpp
)

target_link_libraries(cavr_core_domain_types_test
  PRIVATE
    CAVR::core
)

add_test(NAME cavr_core_domain_types_test COMMAND cavr_core_domain_types_test)
