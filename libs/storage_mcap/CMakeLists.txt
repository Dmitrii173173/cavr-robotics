option(CAVR_ENABLE_MCAP "Build the MCAP storage backend (vendored foxglove/mcap)" ON)

# When the backend is disabled, keep the historical header-only stub so the rest
# of the tree configures unchanged.
if(NOT CAVR_ENABLE_MCAP)
  add_library(cavr_storage_mcap INTERFACE)
  add_library(CAVR::storage_mcap ALIAS cavr_storage_mcap)
  target_include_directories(cavr_storage_mcap
    INTERFACE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(cavr_storage_mcap INTERFACE CAVR::core)
  return()
endif()

# The MCAP backend is CAVR's only compiled library and only third-party
# dependency. The vendored MCAP headers are compiled in this single translation
# unit with compression disabled, so no external libraries (lz4/zstd) are needed.
add_library(cavr_storage_mcap STATIC
  src/mcap_recording.cpp
)
add_library(CAVR::storage_mcap ALIAS cavr_storage_mcap)

target_include_directories(cavr_storage_mcap
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Vendored MCAP headers are an internal implementation detail (SYSTEM keeps their
# warnings out of our build, PRIVATE keeps them off our consumers).
target_include_directories(cavr_storage_mcap SYSTEM
  PRIVATE
    ${CMAKE_SOURCE_DIR}/third_party/mcap/include
)

target_compile_definitions(cavr_storage_mcap
  PRIVATE
    MCAP_COMPRESSION_NO_LZ4
    MCAP_COMPRESSION_NO_ZSTD
)

target_compile_features(cavr_storage_mcap PUBLIC cxx_std_20)

target_link_libraries(cavr_storage_mcap
  PUBLIC
    CAVR::record
    CAVR::core
)
