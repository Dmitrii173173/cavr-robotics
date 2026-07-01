add_executable(cavr-inspect main.cpp)

target_link_libraries(cavr-inspect
  PRIVATE
    CAVR::runtime
    CAVR::record
    CAVR::machine
    CAVR::core
)

if(CAVR_ENABLE_MCAP)
  target_link_libraries(cavr-inspect PRIVATE CAVR::storage_mcap)
  target_compile_definitions(cavr-inspect PRIVATE CAVR_WITH_MCAP)
endif()
