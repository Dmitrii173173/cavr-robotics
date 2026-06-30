add_library(cavr_runtime INTERFACE)
add_library(CAVR::runtime ALIAS cavr_runtime)

target_include_directories(cavr_runtime
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cavr_runtime
  INTERFACE
    CAVR::core
    CAVR::machine
    CAVR::adapter_sdk
    CAVR::validation
    CAVR::record   # SessionLog <-> recording bridge (record_session.hpp)
    CAVR::catalog  # recording -> CatalogSession indexer (catalog_index.hpp)
)
