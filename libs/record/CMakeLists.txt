add_library(cavr_record INTERFACE)
add_library(CAVR::record ALIAS cavr_record)

target_include_directories(cavr_record
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cavr_record
  INTERFACE
    CAVR::core
    CAVR::time
    CAVR::adapter_sdk
)
