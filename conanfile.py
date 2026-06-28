from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain, cmake_layout


class TwinReplayConan(ConanFile):
    name = "twinreplay"
    version = "0.1.0"
    package_type = "application"

    settings = "os", "compiler", "build_type", "arch"

    exports_sources = (
        "CMakeLists.txt",
        "CMakePresets.json",
        "cmake/*",
        "libs/*",
        "adapters/*",
        "apps/*",
        "tests/*",
        "schemas/*",
    )

    def layout(self):
        cmake_layout(self)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()

        toolchain = CMakeToolchain(self)
        toolchain.generate()
