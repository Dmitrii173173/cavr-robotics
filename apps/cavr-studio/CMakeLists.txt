find_package(Qt6 QUIET COMPONENTS Widgets OpenGLWidgets)

if(NOT Qt6_FOUND)
  message(WARNING "Qt 6 Widgets was not found. The cavr-studio desktop application will not be built.")
  return()
endif()

add_executable(cavr-studio
  src/CameraView.cpp
  src/CameraView.hpp
  src/SceneViewport.cpp
  src/SceneViewport.hpp
  src/StudioWindow.cpp
  src/StudioWindow.hpp
  src/TimelineWidget.cpp
  src/TimelineWidget.hpp
  src/main.cpp
)

target_link_libraries(cavr-studio
  PRIVATE
    Qt6::Widgets
    Qt6::OpenGLWidgets
    CAVR::visualization
)

target_compile_features(cavr-studio PRIVATE cxx_std_20)

set_target_properties(cavr-studio PROPERTIES
  WIN32_EXECUTABLE TRUE
  MACOSX_BUNDLE FALSE
)
