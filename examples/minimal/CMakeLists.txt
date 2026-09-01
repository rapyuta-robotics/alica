cmake_minimum_required(VERSION 3.10)
project(minimal_alica CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Only three ALICA packages are needed for a non-ROS, statically-wired agent.
find_package(alica_solver_interface REQUIRED)
find_package(alica_engine REQUIRED)
find_package(alica_dummy_proxy REQUIRED)

add_executable(minimal_alica
  src/main.cpp
  src/CountUp.cpp
)

target_link_libraries(minimal_alica PRIVATE
  alica_engine
  alica_dummy_proxy
  alica_solver_interface
)

target_include_directories(minimal_alica PRIVATE src)

# Optional Tracy instrumentation.
#
# ON in the top-level tree comes from the parent's cache entry, which this
# option() call then leaves alone; standalone builds pass it themselves. The
# imported target carries -DTRACY_ENABLE, which is what src/Profiling.h keys
# off, so nothing else has to be defined here.
option(ALICA_BUILD_PROFILE "Instrument for the Tracy profiler" OFF)
if(ALICA_BUILD_PROFILE)
  find_package(Tracy REQUIRED)

  # Retrieve the interface compile definitions from the Tracy Conan target
  get_target_property(TRACY_DEFS Tracy::TracyClient INTERFACE_COMPILE_DEFINITIONS)
  message(STATUS "Tracy compile definitions: ${TRACY_DEFS}")
  # Check if TRACY_ENABLE exists in that list
  if(NOT TRACY_DEFS MATCHES "TRACY_ENABLE")
    message(FATAL_ERROR 
      "\n[ALICA Error]: ALICA_BUILD_PROFILE is enabled, but the Tracy library "
      "provided by Conan was built without profiling enabled!\n"
      "Please re-install dependencies with the option 'tracy/*:enable=True' "
      "in your conanfile or profile.\n"
    )
  endif()

  target_link_libraries(minimal_alica PRIVATE Tracy::TracyClient)
endif()
