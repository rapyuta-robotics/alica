#
# Conan recipe for the ALICA framework.
#
# Building ALICA itself, with its third-party dependencies from Conan:
#
#   conan install . --build=missing
#   cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake
#   cmake --build build -j"$(nproc)"
#   cmake --build build --target check
#
# Packaging ALICA for another project to consume:
#
#   conan create . --build=missing -o "&:with_tests=False"
#
# then, in the consumer's conanfile.py, self.requires("alica/1.1.0"), and in
# its CMakeLists.txt:
#
#   find_package(alica REQUIRED)
#   target_link_libraries(app PRIVATE alica::engine alica::dummy_proxy)
#
# The first flow exists to supply ALICA's three third-party dependencies --
# yaml-cpp, Boost and GTest -- on hosts where installing them system-wide is
# not wanted. It drives the same top-level CMakeLists.txt that a plain
# `cmake -S . -B build` drives; nothing here changes how the project builds.
#
# The second installs that same build and repackages the dozen
# find_package()-able packages it produces as one Conan package, with one
# component per library (see package_info()). ALICA's own <pkg>Config.cmake
# files are not shipped: they do not declare their yaml-cpp or Boost
# dependencies, so they would not work outside this build anyway.
#
# ROS 2 is opt-in, with -o "&:with_ros2=True". It takes ROS 2 Kilted from the
# ros-conan recipes (https://github.com/conan-io/ros-conan), which have to be
# registered as a remote first:
#
#   conan remote add ros-conan <ros-conan checkout> --type=local-recipes-index
#   conan create . --build=missing -pr:a <ros-conan checkout>/profiles/ros \
#       -o "&:with_tests=False" -o "&:with_ros2=True"
#
# That adds alica_msgs and alica_ros2_proxy to the package, plus
# alica_ros2_turtlesim if ROS provides turtlesim (ros-kilted's "desktop"
# variant). They are ament packages, so a consumer finds them the ROS way --
# find_package(alica_ros2_proxy) -- rather than as alica:: components; the
# package puts itself on AMENT_PREFIX_PATH and CMAKE_PREFIX_PATH for that. The
# consumer has to require ros-kilted itself as well (see requirements()).
# Without with_ros2 the package is ROS-free whatever the host.
#
# On versions: yaml-cpp is 0.8.0 because ros-kilted requires exactly that, and
# one pin for every configuration keeps ROS and non-ROS builds parsing YAML the
# same way. It is newer than the 0.7.0 Ubuntu 22.04 ships to rosdep builds;
# ALICA uses none of what changed between the two. Boost and GTest are newer
# than jammy's too -- ConanCenter's oldest GTest recipe already postdates
# jammy's 1.11, and ALICA touches only Boost.DLL's header-only alias macro plus
# Boost.Filesystem and Boost.System, whose interfaces have been stable across
# the whole range.
#

import glob
import os

from conan import ConanFile
from conan.tools.build import check_min_cppstd
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy, rmdir


class AlicaConan(ConanFile):
    name = "alica"
    version = "1.1.0"
    description = "ALICA multi-agent coordination framework"
    homepage = "https://github.com/rapyuta-robotics/alica"
    license = "MIT"
    # Most libraries are SHARED unconditionally. alica_simple_solver and
    # alica_dynamic_loading follow BUILD_SHARED_LIBS, which nothing sets, so
    # they come out static -- see the Boost requirement below.
    package_type = "shared-library"

    settings = "os", "arch", "compiler", "build_type"

    # These mirror the ALICA_* options in the top-level CMakeLists.txt. They
    # are declared here as well as there because they decide what has to be
    # downloaded: with_tests=False means GTest is never fetched, and
    # with_ros2=False means ROS is never built.
    options = {
        "with_supplementary": [True, False],
        "with_tests": [True, False],
        "with_ros2": [True, False],
    }
    default_options = {
        "with_supplementary": True,
        "with_tests": True,
        "with_ros2": False,
    }

    # What `conan create` copies into the cache. ROS 1 is left out: build()
    # always turns it off, so those packages are never configured, and leaving
    # them out keeps the recipe revision from changing on edits that cannot
    # affect the package.
    exports_sources = (
        "CMakeLists.txt",
        "LICENSE",
        "cmake_flags/*",
        "alica_*/*",
        "supplementary/alica_dummy_tracing/*",
        "supplementary/autodiff/*",
        "supplementary/constraintsolver/*",
        "supplementary/alica_msgs/*",
        "supplementary/alica_ros2/*",
    )

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
        #
        # The visibility traits are for consumers of the package. alica_engine's
        # public headers hold YAML::Node members, alica_standard_library's
        # include Boost, and alica_test_utility's include gtest, so consumers
        # need those headers. They need the libraries too: yaml-cpp and Boost
        # are static (their recipes' default) and some of their symbols are
        # only referenced from those headers, and the static
        # alica_dynamic_loading leaves its Boost references for the final link.
        self.requires("yaml-cpp/0.8.0", transitive_headers=True, transitive_libs=True)
        self.requires("boost/1.83.0", transitive_headers=True, transitive_libs=True)

        if self.options.with_tests:
            self.requires("gtest/1.14.0", transitive_headers=True, transitive_libs=True)

        if self.options.with_ros2:
            # Not visible to consumers. ros-kilted generates no CMakeDeps files,
            # yet CMakeDeps would still emit find_dependency(ros-kilted) into
            # alica-config.cmake for a visible one, which then fails. A consumer
            # of the ROS parts requires ros-kilted itself instead, as ros-conan's
            # own examples do -- it needs ROS's environment for
            # find_package(rclcpp) regardless.
            self.requires("ros-kilted/2026.06.17", visible=False)

    def validate(self):
        check_min_cppstd(self, 17)

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

        # Installed libraries otherwise carry no RUNPATH, so one shared ALICA
        # library could find another only through the consumer's own RUNPATH or
        # LD_LIBRARY_PATH. They all install to the same directory. This goes in
        # the toolchain rather than on the command line in build(), where the
        # shell would expand $ORIGIN to nothing.
        tc.variables["CMAKE_INSTALL_RPATH"] = "$ORIGIN"

        tc.generate()

    def build(self):
        cmake = CMake(self)
        # Passed on the command line rather than through the toolchain, so they
        # apply to `conan build` and `conan create` only. A hand-run cmake
        # against the toolchain keeps the CMakeLists.txt's ROS auto-detection.
        cmake.configure(variables={
            # The top-level CMakeLists.txt enables these whenever a ROS
            # environment is sourced, which would make the package's contents
            # depend on the shell `conan create` happened to run in. ros-kilted's
            # own environment sets ROS_DISTRO too, so ROS 2 follows the option.
            "ALICA_WITH_ROS1": False,
            "ALICA_WITH_ROS2": bool(self.options.with_ros2),
        })
        cmake.build()

    def package(self):
        copy(self, "LICENSE", self.source_folder, os.path.join(self.package_folder, "licenses"))
        cmake = CMake(self)
        cmake.install()
        # Each package installs its CMake config to lib/<pkg>/cmake. Conan
        # generates the consumer's config from package_info() instead. The ROS
        # packages' ament configs under share/ stay: they are how a consumer
        # finds those. alica_ros2_proxy installs one of each, and CMake would
        # otherwise prefer the lib/ one, which does not find rclcpp.
        for path in glob.glob(os.path.join(self.package_folder, "lib", "*", "cmake")):
            rmdir(self, path)
        for entry in os.listdir(os.path.join(self.package_folder, "lib")):
            path = os.path.join(self.package_folder, "lib", entry)
            if os.path.isdir(path) and not os.listdir(path):
                rmdir(self, path)

    def package_info(self):
        # One find_package(alica) for the lot, with a component per library,
        # named after the library without its alica_ prefix.
        self.cpp_info.set_property("cmake_file_name", "alica")
        self.cpp_info.set_property("cmake_target_name", "alica::alica")

        def component(name, lib, requires=()):
            c = self.cpp_info.components[name]
            c.set_property("cmake_target_name", f"alica::{name}")
            c.libs = [lib] if lib else []
            c.requires = list(requires)
            return c

        component("solver_interface", "alica_solver_interface")

        engine = component("engine", "alica_engine", ["solver_interface", "yaml-cpp::yaml-cpp"])
        if self.settings.os in ("Linux", "FreeBSD"):
            engine.system_libs = ["pthread"]

        component("dummy_proxy", "alica_dummy_proxy", ["engine"])
        component("simple_solver", "alica_simple_solver", ["engine", "solver_interface"])
        component("standard_library", "alica_standard_library",
                  ["engine", "solver_interface", "boost::headers"])
        component("dummy_tracing", None, ["engine"])

        dynamic_loading = component("dynamic_loading", "alica_dynamic_loading",
                                    ["engine", "solver_interface", "boost::system", "boost::filesystem"])
        if self.settings.os in ("Linux", "FreeBSD"):
            dynamic_loading.system_libs = ["dl"]

        if self.options.with_supplementary:
            component("autodiff", "autodiff", ["solver_interface"])
            component("constraintsolver", "constraintsolver",
                      ["autodiff", "engine", "yaml-cpp::yaml-cpp"])

        if self.options.with_tests:
            component("test_utility", "alica_test_utility",
                      ["engine", "dummy_proxy", "solver_interface", "gtest::libgtest"])

        if self.options.with_ros2:
            # The ament packages are found through the environment, as
            # ros-kilted's own are. LD_LIBRARY_PATH is for the message
            # typesupport libraries, which rosidl dlopen()s by searching it.
            #
            # No component requires ros-kilted: it exposes no CMakeDeps targets
            # to require (see requirements()).
            self.cpp_info.ignored_requires = ["ros-kilted"]
            p = self.package_folder
            for env in (self.buildenv_info, self.runenv_info):
                env.prepend_path("AMENT_PREFIX_PATH", p)
                env.prepend_path("CMAKE_PREFIX_PATH", p)
                for site in glob.glob(os.path.join(p, "lib", "python*", "site-packages")):
                    env.prepend_path("PYTHONPATH", site)
            self.runenv_info.prepend_path("LD_LIBRARY_PATH", os.path.join(p, "lib"))
