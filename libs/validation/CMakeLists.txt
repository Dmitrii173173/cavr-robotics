add_library(twinreplay_validation INTERFACE)
add_library(TwinReplay::validation ALIAS twinreplay_validation)

target_include_directories(twinreplay_validation
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(twinreplay_validation
  INTERFACE
    TwinReplay::core
    TwinReplay::time
)
