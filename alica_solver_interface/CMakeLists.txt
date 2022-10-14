cmake_minimum_required(VERSION 3.10)
project(alica_solver_interface)

## Change to use clang ... unfortunately it doesnt work :(
#set(CMAKE_CXX_COMPILER "clang")
#
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
  set(CMAKE_CXX_COMPILER_ARG1 "-std=c++14" CACHE STRING "C++ version for eclipse" FORCE)
  set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
  add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

# Set up utils to allow Colcon/catkin to find the packages
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake) 


###########
## Build ##
###########

include_directories(include/${PROJECT_NAME})

add_library(alica_solver_interface SHARED
 src/SolverTerm.cpp
 src/SolverVariable.cpp
)

target_include_directories(alica_solver_interface PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(alica_solver_interface PUBLIC)


# Testing Setup
set(THREADS_PREFER_PTHREAD_FLAG ON)

include(CTest)
find_package(Threads REQUIRED)
find_package(GTest REQUIRED)
  include_directories(BEFORE
  ${GTEST_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}-tests
  test/test_intervals.cpp
  test/test_main.cpp
)

target_link_libraries(${PROJECT_NAME}-tests PRIVATE Threads::Threads ${PROJECT_NAME} ${GTEST_LIBRARIES})

add_test(NAME IntervalTest.Double COMMAND ${PROJECT_NAME}-tests --gtest_filter=IntervalTest.Double --gtest_also_run_disabled_tests)
add_test(NAME IntervalTest.Operations COMMAND ${PROJECT_NAME}-tests --gtest_filter=IntervalTest.Operations --gtest_also_run_disabled_tests)



install(TARGETS ${PROJECT_NAME}
  EXPORT ${CMAKE_PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)


# Export targets to make package libraries
export(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake
)

install(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_PROJECT_NAME}Targets.cmake
  DESTINATION ${ConfigPackageLocation}
)


# Installing files to make a find_package()-able package
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  VERSION 0.9.4
  COMPATIBILITY SameMajorVersion
)

set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})

configure_package_config_file(
  ${CMAKE_PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${ConfigPackageLocation}
  PATH_VARS INCLUDE_INSTALL_DIR
)

install(FILES 
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${ConfigPackageLocation}
)
