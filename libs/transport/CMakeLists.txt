add_library(cavr_transport INTERFACE)
add_library(CAVR::transport ALIAS cavr_transport)

target_include_directories(cavr_transport
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cavr_transport
  INTERFACE
    CAVR::core
)
