add_executable(cavr-play main.cpp)

target_link_libraries(cavr-play
  PRIVATE
    CAVR::replay
)
