add_executable(cavr-convert main.cpp)

target_link_libraries(cavr-convert
  PRIVATE
    CAVR::record
    CAVR::core
)

if(CAVR_ENABLE_MCAP)
  target_link_libraries(cavr-convert PRIVATE CAVR::storage_mcap)
  target_compile_definitions(cavr-convert PRIVATE CAVR_WITH_MCAP)
endif()
