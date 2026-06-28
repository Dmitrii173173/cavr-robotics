#include "StudioWindow.hpp"

#include <QApplication>
#include <QCoreApplication>
#include <iostream>
#include <string_view>

int main(int argc, char* argv[]) {
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument(argv[index]);
    if (argument == "--help" || argument == "-h") {
      std::cout << "CAVR Studio Qt shell\n";
      std::cout << "Usage: cavr-studio [--help] [--version]\n";
      return 0;
    }
    if (argument == "--version") {
      std::cout << "CAVR Studio 0.1.0\n";
      return 0;
    }
  }

  QApplication application(argc, argv);
  QCoreApplication::setApplicationName("CAVR Studio");
  QCoreApplication::setApplicationVersion("0.1.0");

  StudioWindow window;
  window.show();
  return application.exec();
}
