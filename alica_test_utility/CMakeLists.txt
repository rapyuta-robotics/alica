cmake_minimum_required(VERSION 3.10.2)
project(alica_test_utility)

add_compile_options(-std=c++17)

include(../cmake_flags/cflags.cmake)
alica_compile_flags()

# Set up utils to allow Colcon/catkin to find the packages
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/cmake)

# Imports
find_package(alica_engine REQUIRED)
find_package(alica_dummy_proxy REQUIRED)
find_package(alica_solver_interface REQUIRED)

find_package(GTest REQUIRED)

if(TARGET GTest::gtest)
  set(alica_test_utility_GTEST GTest::gtest)
else()
  set(alica_test_utility_GTEST ${GTEST_LIBRARIES})
endif()

# Build
include_directories(include)

add_library(${PROJECT_NAME} SHARED
	src/TestBehaviourCreator.cpp
	src/TestContext.cpp
	src/IdleBehaviour.cpp
	src/Util.cpp
)

target_sources(${PROJECT_NAME} PRIVATE
  src/TestBehaviourCreator.cpp
  src/TestContext.cpp
  src/IdleBehaviour.cpp
  src/Util.cpp
  src/CounterClass.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  alica_engine
  alica_dummy_proxy
  alica_solver_interface
  ${alica_test_utility_GTEST}
)

if(GTEST_INCLUDE_DIRS)
  target_include_directories(${PROJECT_NAME} PUBLIC ${GTEST_INCLUDE_DIRS})
endif()

# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Export targets to make package libraries
export(EXPORT ${PROJECT_NAME}Targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION ${ConfigPackageLocation}
)

# Installing files to make a find_package()-able package
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION 1.0.0
  COMPATIBILITY SameMajorVersion
)

set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})

configure_package_config_file(
  ${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${ConfigPackageLocation}
  PATH_VARS INCLUDE_INSTALL_DIR
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${ConfigPackageLocation}
)
