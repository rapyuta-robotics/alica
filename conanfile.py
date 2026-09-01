#
# Conan recipe for the ALICA framework.
#
#   conan install . --build=missing
#   cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan/conan_toolchain.cmake
#   cmake --build build -j"$(nproc)"
#   cmake --build build --target check
#
# This recipe exists to supply ALICA's three third-party dependencies --
# yaml-cpp, Boost and GTest -- on hosts where installing them system-wide is
# not wanted. It drives the same top-level CMakeLists.txt that a plain
# `cmake -S . -B build` drives; nothing here changes how the project builds.
#
# It is deliberately NOT a recipe that packages ALICA for others to consume:
# there is no package()/package_info(), because ALICA installs a dozen
# separate find_package()-able packages rather than one, and upstream ships
# those through catkin/colcon. `conan create` is not a supported flow.
#
# On versions: yaml-cpp is pinned to 0.7.0, the version Ubuntu 22.04 ships, so
# that a Conan build and the rosdep build CI runs parse the same YAML. Boost
# and GTest are newer than jammy's -- ConanCenter's oldest GTest recipe already
# postdates jammy's 1.11, and ALICA touches only Boost.DLL's header-only alias
# macro plus Boost.Filesystem and Boost.System, whose interfaces have been
# stable across the whole range.
#

from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout


class AlicaConan(ConanFile):
    name = "alica"
    version = "1.1.0"
    description = "ALICA multi-agent coordination framework"
    homepage = "https://github.com/rapyuta-robotics/alica"
    license = "MIT"

    settings = "os", "arch", "compiler", "build_type"

    # These mirror the ALICA_* options in the top-level CMakeLists.txt. They
    # are declared here as well as there because they decide what has to be
    # downloaded: with_tests=False means GTest is never fetched.
    options = {
        "with_supplementary": [True, False],
        "with_tests": [True, False],
        "with_examples": [True, False],
        "with_profile": [True, False],
    }
    default_options = {
        "with_supplementary": True,
        "with_tests": True,
        "with_examples": True,
        "with_profile": True,
    }

    def layout(self):
        # Keep `cmake -S . -B build` -- the command the top-level CMakeLists.txt
        # documents -- working verbatim. cmake_layout() would otherwise insert a
        # per-build-type directory (build/Debug) and put the toolchain inside
        # it, so the generators go in a sibling directory instead. Conan writes
        # them before CMake exists, and CMake leaves unknown subdirectories of
        # its binary directory alone, so the two coexist.
        cmake_layout(self, build_folder="build")
        self.folders.build = "build"
        self.folders.generators = "build"

    def requirements(self):
        # See the note on versions at the top of this file.
        self.requires("yaml-cpp/0.7.0")
        self.requires("boost/1.83.0")

        if self.options.with_tests:
            self.requires("gtest/1.14.0")

        if self.options.with_profile:
            # Client library only -- the recipe does not package the server
            # (the tracy-profiler GUI or tracy-capture), which has to be built
            # from Tracy's own sources. See examples/minimal/README.md.
            #
            # Note for instrumenting ALICA itself rather than just an example:
            # TracyClient defaults to a static library, which would give the
            # engine .so and the executable a profiler instance each. Add
            # `-o "tracy/*:shared=True"` so they share one.
            self.requires("tracy/0.13.1")

    def configure(self):
        self.options["tracy"].shared = True
        self.options["tracy/*"].enable = True

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()

        tc = CMakeToolchain(self)

        # Pass the options through under the names the CMake side knows.
        #
        # These go in tc.variables, not tc.cache_variables. cache_variables are
        # written only to CMakePresets.json, so they reach CMake solely through
        # `cmake --preset`, which needs CMake 3.23; on anything older the
        # options would be silently ignored and the build would quietly
        # disagree with the recipe. tc.variables are written into
        # conan_toolchain.cmake itself as cache entries, so they apply however
        # the toolchain is handed to CMake.
        tc.variables["ALICA_BUILD_SUPPLEMENTARY"] = bool(self.options.with_supplementary)
        tc.variables["ALICA_BUILD_TESTS"] = bool(self.options.with_tests)
        tc.variables["ALICA_BUILD_EXAMPLES"] = bool(self.options.with_examples)
        tc.variables["ALICA_BUILD_PROFILE"] = bool(self.options.with_profile)

        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
