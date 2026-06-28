add_library(twinreplay_transport INTERFACE)
add_library(TwinReplay::transport ALIAS twinreplay_transport)

target_include_directories(twinreplay_transport
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(twinreplay_transport
  INTERFACE
    TwinReplay::core
)
