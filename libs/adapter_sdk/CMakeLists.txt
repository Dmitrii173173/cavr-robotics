add_library(cavr_adapter_sdk INTERFACE)
add_library(CAVR::adapter_sdk ALIAS cavr_adapter_sdk)

target_include_directories(cavr_adapter_sdk
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cavr_adapter_sdk
  INTERFACE
    CAVR::core
)
