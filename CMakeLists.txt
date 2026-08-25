#
# Top-level superbuild for the ALICA framework.
#
#   cmake -S . -B build
#   cmake --build build -j"$(nproc)"
#
# This file compiles nothing itself. Each ALICA package stays an independent
# CMake project -- they locate one another with find_package() and derive their
# install/export paths from ${CMAKE_PROJECT_NAME} -- so they cannot simply be
# add_subdirectory()'d into one tree without colliding. Instead each is
# configured, built and installed in dependency order via ExternalProject_Add,
# which keeps every package byte-for-byte compatible with a standalone cmake,
# catkin or colcon build.
#
# ROS components are selected from the host platform: ROS 1 on Ubuntu 20.04,
# ROS 2 on Ubuntu 22.04 and later, and neither on non-Linux hosts. See
# "Platform detection" below; every choice can be overridden explicitly.
#

cmake_minimum_required(VERSION 3.16)

project(alica_superbuild
  VERSION 1.1.0
  DESCRIPTION "Superbuild for the ALICA multi-agent coordination framework"
  HOMEPAGE_URL "https://github.com/rapyuta-robotics/alica"
  LANGUAGES C CXX)

include(ExternalProject)

# ---------------------------------------------------------------------------
# Install prefix
# ---------------------------------------------------------------------------
# Packages must be installed before their dependents can configure, so the
# build always installs. Default to a prefix inside the build tree so that a
# plain `cmake --build` never needs root; pass -DCMAKE_INSTALL_PREFIX=... to
# stage somewhere else.

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
# sub-builds fail at configure time. Detect a sourced workspace.
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

set(ALICA_EXTRA_CMAKE_ARGS "" CACHE STRING
    "Extra -D arguments forwarded verbatim to every ALICA package")

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
    "sub-builds will fail to find catkin/ament_cmake.")
endif()
if(ALICA_WITH_ROS1 AND ALICA_ROS2_ELIGIBLE)
  message(WARNING
    "ALICA_WITH_ROS1 is ON but this host looks like a ROS 2 platform "
    "(Ubuntu ${ALICA_DISTRO_VERSION}). ROS 1 Noetic is not available past Ubuntu 20.04.")
endif()

# ---------------------------------------------------------------------------
# Arguments forwarded to every package
# ---------------------------------------------------------------------------
# The install prefix goes on the front of CMAKE_PREFIX_PATH so each package
# finds the ones built before it, while still honouring the caller's own
# CMAKE_PREFIX_PATH for third-party dependencies (yaml-cpp, Boost, GTest, ROS).

set(_alica_prefix_path "${CMAKE_INSTALL_PREFIX}")
if(CMAKE_PREFIX_PATH)
  list(APPEND _alica_prefix_path ${CMAKE_PREFIX_PATH})
endif()
if(DEFINED ENV{CMAKE_PREFIX_PATH} AND NOT "$ENV{CMAKE_PREFIX_PATH}" STREQUAL "")
  string(REPLACE ":" ";" _alica_env_prefix "$ENV{CMAKE_PREFIX_PATH}")
  list(APPEND _alica_prefix_path ${_alica_env_prefix})
endif()
list(REMOVE_DUPLICATES _alica_prefix_path)
# ExternalProject splits CMAKE_ARGS on ';', so lists travel with a stand-in
# separator declared per-project via LIST_SEPARATOR.
string(REPLACE ";" "|" _alica_prefix_path_arg "${_alica_prefix_path}")

set(_alica_common_args
  "-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}"
  "-DCMAKE_PREFIX_PATH:STRING=${_alica_prefix_path_arg}"
  "-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}"
  "-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}"
  "-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON"
)

# Forward these only when the caller actually set them, so an empty string
# never clobbers a package's own default.
foreach(_var
    CMAKE_BUILD_TYPE
    CMAKE_C_FLAGS CMAKE_CXX_FLAGS
    CMAKE_EXE_LINKER_FLAGS CMAKE_SHARED_LINKER_FLAGS CMAKE_MODULE_LINKER_FLAGS
    CMAKE_TOOLCHAIN_FILE
    CMAKE_EXPORT_COMPILE_COMMANDS
    BOOST_ROOT)
  if(DEFINED ${_var} AND NOT "${${_var}}" STREQUAL "")
    list(APPEND _alica_common_args "-D${_var}=${${_var}}")
  endif()
endforeach()

if(ALICA_BUILD_TESTS)
  list(APPEND _alica_common_args "-DBUILD_TESTING:BOOL=ON")
else()
  list(APPEND _alica_common_args "-DBUILD_TESTING:BOOL=OFF")
endif()

if(ALICA_EXTRA_CMAKE_ARGS)
  list(APPEND _alica_common_args ${ALICA_EXTRA_CMAKE_ARGS})
endif()

# ---------------------------------------------------------------------------
# alica_add_package(<name> PATH <dir> [NO_INSTALL] [DEPENDS ...] [CMAKE_ARGS ...])
# ---------------------------------------------------------------------------
#
# NO_INSTALL is for leaf packages that define no install() rules -- executables
# meant to be run from the build tree, such as examples/minimal. Without it
# ExternalProject would invoke a non-existent `install` target and fail.

function(alica_add_package name)
  cmake_parse_arguments(AAP "NO_INSTALL;HAS_TESTS" "PATH" "DEPENDS;CMAKE_ARGS" ${ARGN})

  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${AAP_PATH}/CMakeLists.txt")
    message(FATAL_ERROR "No CMakeLists.txt under '${AAP_PATH}' for package '${name}'")
  endif()

  if(AAP_HAS_TESTS AND ALICA_BUILD_TESTS)
    set_property(GLOBAL APPEND PROPERTY ALICA_TEST_PACKAGES "${name}")
  endif()

  # A literal empty INSTALL_COMMAND cannot be forwarded through a variable --
  # CMake drops empty elements on list expansion, leaving a valueless keyword.
  # Substitute an explicit no-op command instead.
  set(_install_args "")
  if(AAP_NO_INSTALL)
    set(_install_args INSTALL_COMMAND "${CMAKE_COMMAND}" -E true)
  endif()

  ExternalProject_Add(${name}
    SOURCE_DIR     "${CMAKE_CURRENT_SOURCE_DIR}/${AAP_PATH}"
    BINARY_DIR     "${CMAKE_BINARY_DIR}/packages/${name}"
    PREFIX         "${CMAKE_BINARY_DIR}/stamps/${name}"
    LIST_SEPARATOR "|"
    CMAKE_ARGS     ${_alica_common_args} ${AAP_CMAKE_ARGS}
    DEPENDS        ${AAP_DEPENDS}
    # Re-run the sub-build every time so source edits are picked up. The
    # underlying build is incremental, so this stays cheap.
    BUILD_ALWAYS     ON
    DOWNLOAD_COMMAND ""
    UPDATE_COMMAND   ""
    ${_install_args}
  )

  set_property(TARGET ${name} PROPERTY FOLDER "alica")
endfunction()

# ---------------------------------------------------------------------------
# Core packages -- no ROS, no platform assumptions
# ---------------------------------------------------------------------------

alica_add_package(alica_solver_interface PATH alica_solver_interface HAS_TESTS)

alica_add_package(alica_engine           PATH alica_engine
                                         DEPENDS alica_solver_interface)

alica_add_package(alica_dummy_proxy      PATH alica_dummy_proxy
                                         DEPENDS alica_engine)

alica_add_package(alica_simple_solver    PATH alica_simple_solver
                                         DEPENDS alica_engine)

alica_add_package(alica_dynamic_loading  PATH alica_dynamic_loading
                                         DEPENDS alica_engine)

alica_add_package(alica_standard_library PATH alica_standard_library
                                         DEPENDS alica_engine)

alica_add_package(alica_test_utility     PATH alica_test_utility
                                         DEPENDS alica_dummy_proxy)

alica_add_package(alica_dummy_tracing    PATH supplementary/alica_dummy_tracing
                                         DEPENDS alica_engine)

# ---------------------------------------------------------------------------
# Supplementary, ROS-free -- automatic differentiation and the CGSolver
# ---------------------------------------------------------------------------

if(ALICA_BUILD_SUPPLEMENTARY)
  alica_add_package(autodiff         PATH supplementary/autodiff HAS_TESTS
                                     DEPENDS alica_solver_interface)

  alica_add_package(constraintsolver PATH supplementary/constraintsolver HAS_TESTS
                                     DEPENDS autodiff alica_engine)
endif()

# ---------------------------------------------------------------------------
# Test suite
# ---------------------------------------------------------------------------

if(ALICA_BUILD_TESTS)
  alica_add_package(alica_tests PATH alica_tests HAS_TESTS
    DEPENDS alica_test_utility alica_simple_solver alica_dynamic_loading
            alica_standard_library alica_dummy_proxy)
endif()

# ---------------------------------------------------------------------------
# Examples
# ---------------------------------------------------------------------------

if(ALICA_BUILD_EXAMPLES AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/minimal/CMakeLists.txt")
  # Run it from its own directory, where the etc/ model files live:
  #   ./build/packages/minimal_alica/minimal_alica examples/minimal/etc 2
  alica_add_package(minimal_alica PATH examples/minimal NO_INSTALL
                                  DEPENDS alica_dummy_proxy)
endif()

# ---------------------------------------------------------------------------
# ROS 1 components (Ubuntu 20.04)
# ---------------------------------------------------------------------------
# These are catkin packages. Building them here works when a ROS 1 environment
# is sourced, but `catkin build` remains the workflow upstream supports.

if(ALICA_WITH_ROS1)
  alica_add_package(alica_msgs      PATH supplementary/alica_msgs)

  alica_add_package(alica_ros_proxy PATH supplementary/alica_ros1/alica_ros_proxy
                                    DEPENDS alica_engine alica_msgs)

  alica_add_package(alica_tracing   PATH supplementary/alica_tracing
                                    DEPENDS alica_engine)

  if(ALICA_BUILD_SUPPLEMENTARY)
    alica_add_package(alica_turtlesim PATH supplementary/alica_turtlesim
      DEPENDS alica_engine autodiff constraintsolver alica_standard_library)

    alica_add_package(alica_ros_turtlesim PATH supplementary/alica_ros1/alica_ros_turtlesim
      DEPENDS alica_ros_proxy alica_turtlesim alica_dynamic_loading
              alica_standard_library constraintsolver)

    if(ALICA_BUILD_TESTS)
      alica_add_package(supplementary_tests PATH supplementary/alica_ros1/supplementary_tests HAS_TESTS
        DEPENDS alica_ros_proxy alica_test_utility alica_simple_solver
                alica_dynamic_loading constraintsolver autodiff)
    endif()
  endif()
endif()

# ---------------------------------------------------------------------------
# ROS 2 components (Ubuntu 22.04 and later)
# ---------------------------------------------------------------------------

if(ALICA_WITH_ROS2)
  alica_add_package(alica_msgs       PATH supplementary/alica_msgs)

  alica_add_package(alica_ros2_proxy PATH supplementary/alica_ros2/alica_ros2_proxy
                                     DEPENDS alica_engine alica_msgs)

  if(ALICA_BUILD_SUPPLEMENTARY)
    alica_add_package(alica_ros2_turtlesim PATH supplementary/alica_ros2/alica_ros2_turtlesim
      DEPENDS alica_ros2_proxy alica_dynamic_loading constraintsolver)
  endif()
endif()

# ---------------------------------------------------------------------------
# `check` target -- run every suite with the right environment
# ---------------------------------------------------------------------------
#
#   cmake --build build --target check
#
# Deliberately serial. Several multi-agent tests wait on real wall-clock
# deadlines (Alica.yaml TeamTimeOut is 2000 ms), so running them concurrently
# starves them and they time out spuriously. Serial is ~43 s versus ~11 s for
# `ctest -j4`, and it is reliable. Pass -j yourself in a package build
# directory if you want the faster, flakier run.

get_property(_alica_test_packages GLOBAL PROPERTY ALICA_TEST_PACKAGES)

if(ALICA_BUILD_TESTS AND _alica_test_packages)
  # alica_tests dynamically loads libalica-tests.so from its own build tree.
  set(_alica_ld_path "${CMAKE_BINARY_DIR}/packages/alica_tests:${CMAKE_INSTALL_PREFIX}/lib")
  if(DEFINED ENV{LD_LIBRARY_PATH} AND NOT "$ENV{LD_LIBRARY_PATH}" STREQUAL "")
    set(_alica_ld_path "${_alica_ld_path}:$ENV{LD_LIBRARY_PATH}")
  endif()

  # Retry a timed-out test once before calling it a failure. Several
  # multi-agent tests are wall-clock sensitive and flake on a busy machine;
  # re-running them passes in well under a second. Requires CTest 3.17+.
  set(_alica_ctest_flags --output-on-failure --timeout 120)
  if(NOT CMAKE_VERSION VERSION_LESS 3.17)
    list(APPEND _alica_ctest_flags --repeat until-pass:2)
  endif()

  set(_alica_check_cmds "")
  foreach(_pkg IN LISTS _alica_test_packages)
    list(APPEND _alica_check_cmds
      COMMAND "${CMAKE_COMMAND}" -E echo "-- ctest: ${_pkg}"
      COMMAND "${CMAKE_COMMAND}" -E env "LD_LIBRARY_PATH=${_alica_ld_path}"
              "${CMAKE_COMMAND}" -E chdir "${CMAKE_BINARY_DIR}/packages/${_pkg}"
              "${CMAKE_CTEST_COMMAND}" ${_alica_ctest_flags})
  endforeach()

  add_custom_target(check ${_alica_check_cmds}
    COMMENT "Running all ALICA test suites"
    USES_TERMINAL VERBATIM)
  add_dependencies(check ${_alica_test_packages})
endif()

# ---------------------------------------------------------------------------
# Configuration summary
# ---------------------------------------------------------------------------

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
message(STATUS "  ALICA superbuild ${PROJECT_VERSION}")
message(STATUS "  ----------------------------------------------------------")
message(STATUS "  Host              : ${CMAKE_HOST_SYSTEM_NAME} / ${ALICA_DISTRO_ID} ${ALICA_DISTRO_VERSION}")
message(STATUS "  Platform decision : ${ALICA_PLATFORM_REASON}")
message(STATUS "  ROS components    : ${_alica_ros_line}")
message(STATUS "  Supplementary     : ${ALICA_BUILD_SUPPLEMENTARY}")
message(STATUS "  Tests             : ${ALICA_BUILD_TESTS}")
message(STATUS "  Examples          : ${ALICA_BUILD_EXAMPLES}")
message(STATUS "  Install prefix    : ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  ----------------------------------------------------------")
if(NOT ALICA_WITH_ROS1 AND NOT ALICA_WITH_ROS2 AND ALICA_HOST_IS_LINUX AND NOT ALICA_ROS_DETECTED)
  message(STATUS "  Source /opt/ros/<distro>/setup.bash and re-run cmake to enable")
  message(STATUS "  the ROS components, or force them with -DALICA_WITH_ROS1=ON /")
  message(STATUS "  -DALICA_WITH_ROS2=ON.")
  message(STATUS "  ----------------------------------------------------------")
endif()
message(STATUS "  Note: alica_engine and alica_tests pin CMAKE_BUILD_TYPE to Debug")
message(STATUS "  internally, so -DCMAKE_BUILD_TYPE has no effect on those two.")
message(STATUS "")
if(ALICA_BUILD_TESTS AND _alica_test_packages)
  string(REPLACE ";" ", " _alica_test_list "${_alica_test_packages}")
  message(STATUS "  Test suites       : ${_alica_test_list}")
  message(STATUS "  Run them with     : cmake --build <dir> --target check")
  message(STATUS "")
endif()
message(STATUS "  After building, add the install tree to the runtime path:")
message(STATUS "    export LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:\$LD_LIBRARY_PATH")
message(STATUS "")
