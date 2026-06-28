add_library(twinreplay_storage_mcap INTERFACE)
add_library(TwinReplay::storage_mcap ALIAS twinreplay_storage_mcap)

target_include_directories(twinreplay_storage_mcap
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(twinreplay_storage_mcap
  INTERFACE
    TwinReplay::core
)
