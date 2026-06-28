add_library(twinreplay_time INTERFACE)
add_library(TwinReplay::time ALIAS twinreplay_time)

target_include_directories(twinreplay_time
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(twinreplay_time
  INTERFACE
    TwinReplay::core
)
