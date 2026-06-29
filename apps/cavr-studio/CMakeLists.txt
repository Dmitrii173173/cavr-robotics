find_package(Qt6 QUIET COMPONENTS Widgets Quick Quick3D Qml QuickWidgets)

if(NOT Qt6_FOUND)
  message(WARNING "Qt 6 (Widgets, Quick3D) was not found. The cavr-studio desktop application will not be built.")
  return()
endif()

set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOMOC ON)

add_executable(cavr-studio
  src/CameraView.cpp
  src/CameraView.hpp
  src/RobotController.cpp
  src/RobotController.hpp
  src/StudioWindow.cpp
  src/StudioWindow.hpp
  src/TimelineWidget.cpp
  src/TimelineWidget.hpp
  src/main.cpp
  cavr_studio.qrc
)

target_link_libraries(cavr-studio
  PRIVATE
    Qt6::Widgets
    Qt6::Quick
    Qt6::Quick3D
    Qt6::Qml
    Qt6::QuickWidgets
    CAVR::visualization
    CAVR::runtime
    CAVR::machine
    CAVR::validation
    CAVR::adapter_mock_robot
)

target_compile_features(cavr-studio PRIVATE cxx_std_20)

# Location of runtime assets (gp25.glb and friends), resolved at build time.
target_compile_definitions(cavr-studio
  PRIVATE
    CAVR_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets"
)

set_target_properties(cavr-studio PROPERTIES
  WIN32_EXECUTABLE TRUE
  MACOSX_BUNDLE TRUE          # produce cavr-studio.app so macdeployqt can bundle Qt
)
