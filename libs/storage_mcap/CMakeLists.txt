add_library(cavr_storage_mcap INTERFACE)
add_library(CAVR::storage_mcap ALIAS cavr_storage_mcap)

target_include_directories(cavr_storage_mcap
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cavr_storage_mcap
  INTERFACE
    CAVR::core
)
