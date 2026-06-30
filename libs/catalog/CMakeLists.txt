option(CAVR_ENABLE_SQLITE "Build the SQLite catalog backend (vendored sqlite)" ON)

# Header-only types + in-memory catalog are always available. With SQLite disabled
# the module is just those headers; nothing depends on SQLite.
if(NOT CAVR_ENABLE_SQLITE)
  add_library(cavr_catalog INTERFACE)
  add_library(CAVR::catalog ALIAS cavr_catalog)
  target_include_directories(cavr_catalog
    INTERFACE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  return()
endif()

# The SQLite backend is compiled with the vendored amalgamation. sqlite3.c is C, so
# enable the C language just for this target.
enable_language(C)

add_library(cavr_catalog STATIC
  src/sqlite_catalog.cpp
  ${CMAKE_SOURCE_DIR}/third_party/sqlite/sqlite3.c
)
add_library(CAVR::catalog ALIAS cavr_catalog)

target_include_directories(cavr_catalog
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Vendored SQLite header is an internal detail (SYSTEM/PRIVATE: warnings and the
# header stay out of our consumers).
target_include_directories(cavr_catalog SYSTEM
  PRIVATE
    ${CMAKE_SOURCE_DIR}/third_party/sqlite
)

target_compile_features(cavr_catalog PUBLIC cxx_std_20)

# SQLite uses threads, and dlopen for (unused here) extension loading.
find_package(Threads REQUIRED)
target_link_libraries(cavr_catalog
  PUBLIC
    Threads::Threads
    ${CMAKE_DL_LIBS}
)
