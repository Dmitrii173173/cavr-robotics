# A real ControllerAdapter over TCP. The platform socket code is compiled in one
# translation unit (tcp_connection.cpp), so this is a STATIC library rather than
# the header-only INTERFACE adapters — no <winsock2.h>/<sys/socket.h> leaks past
# it. The protocol and controller headers are header-only on top.
add_library(cavr_adapter_generic_tcp_robot STATIC
  src/tcp_connection.cpp
)
add_library(CAVR::adapter_generic_tcp_robot ALIAS cavr_adapter_generic_tcp_robot)

target_include_directories(cavr_adapter_generic_tcp_robot
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(cavr_adapter_generic_tcp_robot PUBLIC cxx_std_20)

target_link_libraries(cavr_adapter_generic_tcp_robot
  PUBLIC
    CAVR::adapter_sdk
    CAVR::machine
)

# Winsock on Windows; BSD sockets need no extra link elsewhere.
if(WIN32)
  target_link_libraries(cavr_adapter_generic_tcp_robot PUBLIC ws2_32)
endif()
