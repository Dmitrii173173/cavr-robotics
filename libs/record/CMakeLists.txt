add_library(twinreplay_record INTERFACE)
add_library(TwinReplay::record ALIAS twinreplay_record)

target_include_directories(twinreplay_record
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(twinreplay_record
  INTERFACE
    TwinReplay::core
    TwinReplay::time
    TwinReplay::adapter_sdk
)
