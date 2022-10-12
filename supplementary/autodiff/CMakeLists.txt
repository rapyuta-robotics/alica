cmake_minimum_required(VERSION 3.5.1)
project(autodiff)

if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++14" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")


# Set up utils to allow Colcon/catkin to find the packages
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake) 


# Imports
find_package(alica_common_config REQUIRED)
find_package(alica_solver_interface REQUIRED)


# Install
include_directories(include/autodiff 
  ${alica_common_config_INCLUDE_DIRS}
  ${alica_solver_interface_INCLUDE_DIRS}
)

add_library(autodiff SHARED
  src/Abs.cpp
  src/And.cpp
  src/Atan2.cpp
  src/Constant.cpp
  src/ConstPower.cpp
  src/ConstraintBuilder.cpp
  src/ConstraintUtility.cpp
  src/Cos.cpp
  src/Exp.cpp
  src/LinSigmoid.cpp
  src/Log.cpp
  src/LTConstraint.cpp
  src/LTEConstraint.cpp
  src/Max.cpp
  src/Min.cpp
  src/Or.cpp
  src/Product.cpp
  src/Reification.cpp
  src/Sigmoid.cpp
  src/Sin.cpp
  src/Sum.cpp
  src/Tape.cpp
  src/Term.cpp
  src/TermBuilder.cpp
  src/TermList.cpp
  src/TermHolder.cpp
  src/TermPower.cpp
  src/TermPtr.cpp
  src/Variable.cpp
)

target_sources(autodiff PRIVATE
  src/Abs.cpp
  src/And.cpp
  src/Atan2.cpp
  src/Constant.cpp
  src/ConstPower.cpp
  src/ConstraintBuilder.cpp
  src/ConstraintUtility.cpp
  src/Cos.cpp
  src/Exp.cpp
  src/LinSigmoid.cpp
  src/Log.cpp
  src/LTConstraint.cpp
  src/LTEConstraint.cpp
  src/Max.cpp
  src/Min.cpp
  src/Or.cpp
  src/Product.cpp
  src/Reification.cpp
  src/Sigmoid.cpp
  src/Sin.cpp
  src/Sum.cpp
  src/Tape.cpp
  src/Term.cpp
  src/TermBuilder.cpp
  src/TermList.cpp
  src/TermHolder.cpp
  src/TermPower.cpp
  src/TermPtr.cpp
  src/Variable.cpp
)

target_include_directories(autodiff PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(autodiff alica_solver_interface)

add_executable(sine_benchmark test/test_benchmark.cpp)

target_link_libraries(sine_benchmark ${PROJECT_NAME} alica_solver_interface)


# AUTODIFF TESTS
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)

find_package(GTest REQUIRED)

include(CTest)

include_directories(BEFORE
  ${GTEST_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}-tests test/test_autodiff.cpp)

target_link_libraries(${PROJECT_NAME}-tests PRIVATE Threads::Threads ${PROJECT_NAME} ${GTEST_LIBRARIES})

add_test(NAME autodiff-tests COMMAND autodiff-tests)


# Install
install(TARGETS ${PROJECT_NAME}
EXPORT ${CMAKE_PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
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
  VERSION 0.5.0
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
