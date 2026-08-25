#
# Top-level build for the ALICA framework.
#
#   cmake -S . -B build
#   cmake --build build -j"$(nproc)"
#   cmake --build build --target check
#
# Every package is add_subdirectory()'d into a single CMake project, so there
# is one build tree: `ctest` at the top level sees every suite, a single
# compile_commands.json covers the whole repository, and a parallel build
# schedules across package boundaries instead of stopping at each one.
#
# Two upstream idioms had to be adapted to make that legal. Both patches live
# in the package CMakeLists.txt files and are no-ops for a standalone cmake,
# catkin or colcon build:
#
#   1. Packages derived their install and export paths from
#      ${CMAKE_PROJECT_NAME}, which names the *top-level* project. Nested,
#      every package would look for the same <parent>Config.cmake.in and
#      declare the same <parent>Targets export set -- a guaranteed collision.
#      They now use ${PROJECT_NAME}, which is the same string when a package
#      is built on its own.
#
#   2. alica_engine and alica_tests pinned CMAKE_BUILD_TYPE to Debug
#      unconditionally, overriding the caller for their subtree. They now do
#      so only when the caller left it empty.
#
# Packages still find each other with find_package(), which ordinarily
# requires them to be installed. Rather than edit 57 call sites, this file
# writes a small redirect config per package into the build tree and points
# <pkg>_DIR at it, so find_package() resolves against the targets already in
# this tree. That is what CMake >= 3.24 does for FetchContent via
# CMAKE_FIND_PACKAGE_REDIRECTS_DIR; doing it by hand keeps CMake 3.16 working.
#
# ROS components are selected from the host platform: ROS 1 on Ubuntu 20.04,
# ROS 2 on Ubuntu 22.04 and later, and neither on non-Linux hosts. See
# "Platform detection" below; every choice can be overridden explicitly.
#

cmake_minimum_required(VERSION 3.16)

project(alica
  VERSION 1.1.0
  DESCRIPTION "ALICA multi-agent coordination framework"
  HOMEPAGE_URL "https://github.com/rapyuta-robotics/alica"
  LANGUAGES C CXX)

# ---------------------------------------------------------------------------
# Output layout
# ---------------------------------------------------------------------------
# One directory for every shared library and one for every executable. The
# engine loads behaviour libraries through LD_LIBRARY_PATH at runtime, so
# collecting them in a single place means one entry rather than one per
# package.

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# One configuration for the whole tree.
#
# This is not cosmetic. install(EXPORT) writes its per-configuration file --
# the one carrying IMPORTED_LOCATION -- from the *top-level* build type. When
# only a subdirectory sets one, that file is never written, and a downstream
# find_package() fails with "IMPORTED_LOCATION not set for imported target".
# alica_engine and alica_tests default to Debug when built on their own, so
# keep Debug as the default here; -DCMAKE_BUILD_TYPE=Release now applies to
# every package instead of being overridden by those two.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build configuration" FORCE)
endif()

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install"
      CACHE PATH "Where the ALICA packages are installed" FORCE)
endif()

# ---------------------------------------------------------------------------
# Platform detection
# ---------------------------------------------------------------------------

set(ALICA_HOST_IS_LINUX FALSE)
set(ALICA_DISTRO_ID "unknown")
set(ALICA_DISTRO_VERSION "0")

if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
  set(ALICA_HOST_IS_LINUX TRUE)
  if(EXISTS "/etc/os-release")
    file(READ "/etc/os-release" _alica_os_release)
    # Anchor on a newline so ID= does not also match ID_LIKE= or VERSION_ID=.
    # VERSION_ID is quoted on Ubuntu, ID is not, so the quotes are optional.
    if("\n${_alica_os_release}" MATCHES "\nID=\"?([A-Za-z0-9._-]+)\"?")
      set(ALICA_DISTRO_ID "${CMAKE_MATCH_1}")
    endif()
    if("\n${_alica_os_release}" MATCHES "\nVERSION_ID=\"?([0-9]+(\\.[0-9]+)*)\"?")
      set(ALICA_DISTRO_VERSION "${CMAKE_MATCH_1}")
    endif()
    unset(_alica_os_release)
  endif()
endif()

# Which ROS generation suits this host?
#   Ubuntu < 22.04  -> ROS 1 (Noetic is the last ROS 1 LTS and stops at 20.04)
#   Ubuntu >= 22.04 -> ROS 2 (Humble and later; Noetic is unavailable there)
#   other Linux     -> whatever a sourced ROS environment reports
#   non-Linux       -> neither
set(ALICA_ROS1_ELIGIBLE FALSE)
set(ALICA_ROS2_ELIGIBLE FALSE)
set(ALICA_PLATFORM_REASON "")

if(NOT ALICA_HOST_IS_LINUX)
  set(ALICA_PLATFORM_REASON
      "host is ${CMAKE_HOST_SYSTEM_NAME}, not Linux -- ROS components disabled")
elseif(ALICA_DISTRO_ID STREQUAL "ubuntu" AND ALICA_DISTRO_VERSION VERSION_LESS 22.04)
  set(ALICA_ROS1_ELIGIBLE TRUE)
  set(ALICA_PLATFORM_REASON "Ubuntu ${ALICA_DISTRO_VERSION} -> ROS 1 components")
elseif(ALICA_DISTRO_ID STREQUAL "ubuntu")
  set(ALICA_ROS2_ELIGIBLE TRUE)
  set(ALICA_PLATFORM_REASON "Ubuntu ${ALICA_DISTRO_VERSION} -> ROS 2 components")
else()
  # Not Ubuntu: follow a sourced ROS environment rather than guessing.
  if("$ENV{ROS_VERSION}" STREQUAL "1")
    set(ALICA_ROS1_ELIGIBLE TRUE)
    set(ALICA_PLATFORM_REASON
        "${ALICA_DISTRO_ID} ${ALICA_DISTRO_VERSION} with ROS_VERSION=1 -> ROS 1 components")
  elseif("$ENV{ROS_VERSION}" STREQUAL "2")
    set(ALICA_ROS2_ELIGIBLE TRUE)
    set(ALICA_PLATFORM_REASON
        "${ALICA_DISTRO_ID} ${ALICA_DISTRO_VERSION} with ROS_VERSION=2 -> ROS 2 components")
  else()
    set(ALICA_PLATFORM_REASON
        "${ALICA_DISTRO_ID} ${ALICA_DISTRO_VERSION} is not Ubuntu and no ROS environment is sourced")
  endif()
endif()

# Being on the right distro is not enough -- ROS must also be present, or the
# packages fail at configure time. Detect a sourced workspace.
set(ALICA_ROS_DETECTED FALSE)
set(ALICA_ROS_DISTRO "none")
if(DEFINED ENV{ROS_DISTRO} AND NOT "$ENV{ROS_DISTRO}" STREQUAL "")
  set(ALICA_ROS_DETECTED TRUE)
  set(ALICA_ROS_DISTRO "$ENV{ROS_DISTRO}")
endif()

set(_alica_ros1_default OFF)
set(_alica_ros2_default OFF)
if(ALICA_ROS1_ELIGIBLE AND ALICA_ROS_DETECTED AND NOT "$ENV{ROS_VERSION}" STREQUAL "2")
  set(_alica_ros1_default ON)
endif()
if(ALICA_ROS2_ELIGIBLE AND ALICA_ROS_DETECTED AND NOT "$ENV{ROS_VERSION}" STREQUAL "1")
  set(_alica_ros2_default ON)
endif()

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------

option(ALICA_BUILD_SUPPLEMENTARY "Build autodiff and the CGSolver constraint solver" ON)
option(ALICA_BUILD_TESTS         "Build the test suites (ctest)"                     ON)
option(ALICA_BUILD_EXAMPLES      "Build examples/minimal"                            ON)
option(ALICA_WITH_ROS1 "Build the ROS 1 components" ${_alica_ros1_default})
option(ALICA_WITH_ROS2 "Build the ROS 2 components" ${_alica_ros2_default})

# Guard against combinations that cannot work.
if(ALICA_WITH_ROS1 AND ALICA_WITH_ROS2)
  message(FATAL_ERROR
    "ALICA_WITH_ROS1 and ALICA_WITH_ROS2 are mutually exclusive -- alica_msgs "
    "generates either catkin or ament interfaces, not both.")
endif()
if((ALICA_WITH_ROS1 OR ALICA_WITH_ROS2) AND NOT ALICA_HOST_IS_LINUX)
  message(FATAL_ERROR
    "ROS components require Linux, but the host is ${CMAKE_HOST_SYSTEM_NAME}.")
endif()
if((ALICA_WITH_ROS1 OR ALICA_WITH_ROS2) AND NOT ALICA_ROS_DETECTED)
  message(WARNING
    "ROS components are enabled but no ROS environment is sourced (ROS_DISTRO is "
    "unset). Source /opt/ros/<distro>/setup.bash before building, or the "
    "packages will fail to find catkin/ament_cmake.")
endif()
if(ALICA_WITH_ROS1 AND ALICA_ROS2_ELIGIBLE)
  message(WARNING
    "ALICA_WITH_ROS1 is ON but this host looks like a ROS 2 platform "
    "(Ubuntu ${ALICA_DISTRO_VERSION}). ROS 1 Noetic is not available past Ubuntu 20.04.")
endif()
if(ALICA_WITH_ROS1 OR ALICA_WITH_ROS2)
  # catkin_package() and ament_package() both expect to own the top of the
  # tree. Nesting them under this project is not something upstream tests.
  message(WARNING
    "The ROS components are being built from this top-level tree. catkin "
    "(ROS 1) and colcon (ROS 2) remain the workflows upstream supports for "
    "them; if this build fails, fall back to the tooling for your ROS "
    "generation. The core, non-ROS packages are unaffected.")
endif()

# ---------------------------------------------------------------------------
# Package declaration
# ---------------------------------------------------------------------------

enable_testing()

set(ALICA_REDIRECT_DIR "${CMAKE_BINARY_DIR}/alica-find-redirects")
file(MAKE_DIRECTORY "${ALICA_REDIRECT_DIR}")

set_property(GLOBAL PROPERTY ALICA_PACKAGES "")

# alica_add_package(<name> [PATH <dir>] [LIBRARIES <lib>...])
#
# Writes a redirect config for <name> so that a later package's
# find_package(<name>) resolves against this build tree, then pulls the
# package in with add_subdirectory(). Declare packages in dependency order:
# the redirect must exist before the first consumer is added.
#
# The redirect only reproduces the variables the real <name>Config.cmake sets
# -- <name>_FOUND, _INCLUDE_DIRS and _LIBRARIES. It deliberately does not
# create imported targets, because add_subdirectory() has already defined the
# real ones under the same names, and consumers link against those directly.
function(alica_add_package name)
  cmake_parse_arguments(AAP "" "PATH" "LIBRARIES" ${ARGN})

  if(NOT AAP_PATH)
    set(AAP_PATH "${name}")
  endif()
  set(_src "${CMAKE_CURRENT_SOURCE_DIR}/${AAP_PATH}")
  if(NOT EXISTS "${_src}/CMakeLists.txt")
    message(FATAL_ERROR "alica_add_package(${name}): no CMakeLists.txt in ${_src}")
  endif()

  if(NOT AAP_LIBRARIES)
    set(AAP_LIBRARIES "${name}")
  endif()

  set(_inc "")
  if(IS_DIRECTORY "${_src}/include")
    set(_inc "${_src}/include")
  endif()

  file(WRITE "${ALICA_REDIRECT_DIR}/${name}Config.cmake"
"# Generated by the ALICA top-level build -- do not edit.
# Resolves find_package(${name}) against the targets in this build tree,
# which add_subdirectory() has already defined, so no install is needed.
set(${name}_FOUND TRUE)
set(${name}_INCLUDE_DIRS \"${_inc}\")
set(${name}_LIBRARIES ${AAP_LIBRARIES})
")

  set(${name}_DIR "${ALICA_REDIRECT_DIR}"
      CACHE PATH "In-tree config for ${name}" FORCE)
  mark_as_advanced(${name}_DIR)

  add_subdirectory("${AAP_PATH}")

  set_property(GLOBAL APPEND PROPERTY ALICA_PACKAGES "${name}")
endfunction()

# ---------------------------------------------------------------------------
# Core packages -- no ROS, no platform assumptions
# ---------------------------------------------------------------------------

alica_add_package(alica_solver_interface)
alica_add_package(alica_engine)
alica_add_package(alica_dummy_proxy)
alica_add_package(alica_simple_solver)
alica_add_package(alica_dynamic_loading)
alica_add_package(alica_standard_library)
alica_add_package(alica_test_utility)
alica_add_package(alica_dummy_tracing PATH supplementary/alica_dummy_tracing)

# ---------------------------------------------------------------------------
# Supplementary, ROS-free -- automatic differentiation and the CGSolver
# ---------------------------------------------------------------------------

if(ALICA_BUILD_SUPPLEMENTARY)
  alica_add_package(autodiff         PATH supplementary/autodiff)
  alica_add_package(constraintsolver PATH supplementary/constraintsolver)
endif()

# ---------------------------------------------------------------------------
# Test suite
# ---------------------------------------------------------------------------

if(ALICA_BUILD_TESTS)
  alica_add_package(alica_tests)
endif()

# ---------------------------------------------------------------------------
# Examples
# ---------------------------------------------------------------------------

if(ALICA_BUILD_EXAMPLES AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/minimal/CMakeLists.txt")
  # Run it from the example directory, where the etc/ model files live:
  #   ./build/bin/minimal_alica examples/minimal/etc 2
  alica_add_package(minimal_alica PATH examples/minimal)
endif()

# ---------------------------------------------------------------------------
# ROS 1 components (Ubuntu 20.04)
# ---------------------------------------------------------------------------

if(ALICA_WITH_ROS1)
  alica_add_package(alica_msgs      PATH supplementary/alica_msgs)
  alica_add_package(alica_ros_proxy PATH supplementary/alica_ros1/alica_ros_proxy)
  alica_add_package(alica_tracing   PATH supplementary/alica_tracing)

  if(ALICA_BUILD_SUPPLEMENTARY)
    alica_add_package(alica_turtlesim PATH supplementary/alica_turtlesim
                                      LIBRARIES libalica-turtlesim)
    alica_add_package(alica_ros_turtlesim PATH supplementary/alica_ros1/alica_ros_turtlesim)

    if(ALICA_BUILD_TESTS)
      alica_add_package(supplementary_tests PATH supplementary/alica_ros1/supplementary_tests)
    endif()
  endif()
endif()

# ---------------------------------------------------------------------------
# ROS 2 components (Ubuntu 22.04 and later)
# ---------------------------------------------------------------------------

if(ALICA_WITH_ROS2)
  alica_add_package(alica_msgs       PATH supplementary/alica_msgs)
  alica_add_package(alica_ros2_proxy PATH supplementary/alica_ros2/alica_ros2_proxy)

  if(ALICA_BUILD_SUPPLEMENTARY)
    alica_add_package(alica_ros2_turtlesim PATH supplementary/alica_ros2/alica_ros2_turtlesim)
  endif()
endif()

# ---------------------------------------------------------------------------
# `check` target -- run every suite with the right environment
# ---------------------------------------------------------------------------
#
#   cmake --build build --target check
#
# One ctest invocation covers every package, because there is one build tree.
#
# Deliberately serial. Several multi-agent tests wait on real wall-clock
# deadlines (Alica.yaml TeamTimeOut is 2000 ms), so running them concurrently
# on a loaded machine starves them and they time out spuriously -- a different
# test each run, all passing on --rerun-failed. Run `ctest -j` in the build
# directory yourself if you want the faster, flakier version.

if(ALICA_BUILD_TESTS)
  set(_alica_ld_path "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
  if(DEFINED ENV{LD_LIBRARY_PATH} AND NOT "$ENV{LD_LIBRARY_PATH}" STREQUAL "")
    set(_alica_ld_path "${_alica_ld_path}:$ENV{LD_LIBRARY_PATH}")
  endif()

  # A stuck test is stuck forever, so the timeout only decides how much time is
  # burned before the retry. The slowest legitimate test is about 3 s, so 60 s
  # is ~20x headroom while capping the cost of a hang, and three attempts
  # survive a run that stalls more than once (observed on a loaded machine --
  # a single retry is not always enough). Requires CTest 3.17+; on older CTest
  # the run is simply strict.
  set(_alica_ctest_flags --output-on-failure --timeout 60)
  if(NOT CMAKE_VERSION VERSION_LESS 3.17)
    list(APPEND _alica_ctest_flags --repeat until-pass:3)
  endif()

  add_custom_target(check
    COMMAND "${CMAKE_COMMAND}" -E env "LD_LIBRARY_PATH=${_alica_ld_path}"
            "${CMAKE_CTEST_COMMAND}" ${_alica_ctest_flags}
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Running all ALICA test suites"
    USES_TERMINAL VERBATIM)
endif()

# ---------------------------------------------------------------------------
# Configuration summary
# ---------------------------------------------------------------------------

get_property(_alica_packages GLOBAL PROPERTY ALICA_PACKAGES)
list(LENGTH _alica_packages _alica_package_count)

if(ALICA_WITH_ROS1 OR ALICA_WITH_ROS2)
  if(ALICA_WITH_ROS1)
    set(_alica_ros_line "ROS 1")
  else()
    set(_alica_ros_line "ROS 2")
  endif()
  if(ALICA_ROS_DETECTED)
    set(_alica_ros_line "${_alica_ros_line} (${ALICA_ROS_DISTRO})")
  else()
    set(_alica_ros_line "${_alica_ros_line} (forced -- no ROS environment sourced)")
  endif()
else()
  set(_alica_ros_line "none")
endif()

message(STATUS "")
message(STATUS "  ALICA ${PROJECT_VERSION}")
message(STATUS "  ----------------------------------------------------------")
message(STATUS "  Host              : ${CMAKE_HOST_SYSTEM_NAME} / ${ALICA_DISTRO_ID} ${ALICA_DISTRO_VERSION}")
message(STATUS "  Platform decision : ${ALICA_PLATFORM_REASON}")
message(STATUS "  ROS components    : ${_alica_ros_line}")
message(STATUS "  Supplementary     : ${ALICA_BUILD_SUPPLEMENTARY}")
message(STATUS "  Tests             : ${ALICA_BUILD_TESTS}")
message(STATUS "  Examples          : ${ALICA_BUILD_EXAMPLES}")
message(STATUS "  Packages          : ${_alica_package_count}")
message(STATUS "  Build type        : ${CMAKE_BUILD_TYPE}")
message(STATUS "  Libraries         : ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
message(STATUS "  Executables       : ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "  ----------------------------------------------------------")
if(NOT ALICA_WITH_ROS1 AND NOT ALICA_WITH_ROS2 AND ALICA_HOST_IS_LINUX AND NOT ALICA_ROS_DETECTED)
  message(STATUS "  Source /opt/ros/<distro>/setup.bash and re-run cmake to enable")
  message(STATUS "  the ROS components, or force them with -DALICA_WITH_ROS1=ON /")
  message(STATUS "  -DALICA_WITH_ROS2=ON.")
  message(STATUS "  ----------------------------------------------------------")
endif()
message(STATUS "")
if(ALICA_BUILD_TESTS)
  message(STATUS "  Run the tests with : cmake --build <dir> --target check")
  message(STATUS "")
endif()
message(STATUS "  To run binaries directly, put the libraries on the load path:")
message(STATUS "    export LD_LIBRARY_PATH=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}:\$LD_LIBRARY_PATH")
message(STATUS "")
