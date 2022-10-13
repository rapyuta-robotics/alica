cmake_minimum_required(VERSION 3.5.1)
project(constraintsolver)

add_compile_options(-std=c++17)


# Set up utils to allow Colcon/catkin to find the packages
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake) 

##Yaml
find_package(yaml-cpp REQUIRED)

# find_package(catkin REQUIRED COMPONENTS
#   # alica_ros_proxy  
#   roslib
# )

# find_package(fsystem REQUIRED)
find_package(alica_common_config REQUIRED)
find_package(alica_solver_interface REQUIRED)
find_package(autodiff REQUIRED)
find_package(alica_engine REQUIRED)

# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES constraintsolver
#  CATKIN_DEPENDS   roslib 
#  DEPENDS alica_common_config alica_solver_interface autodiff alica_engine
# )

include_directories(
  include
  ${alica_common_config_INCLUDE_DIRS}
  ${alica_engine_INCLUDE_DIRS}
)


add_library(constraintsolver SHARED
  src/intervals/DownwardPropagator.cpp
  src/intervals/IntervalPropagator.cpp
  src/intervals/RecursivePropagate.cpp
  src/intervals/ResetIntervals.cpp
  src/intervals/SetParents.cpp
  src/intervals/UnsolveableException.cpp
  src/intervals/UpwardPropagator.cpp
  src/types/Clause.cpp
  src/types/DecisionLevel.cpp
  src/types/Lit.cpp
  src/types/Var.cpp
  src/types/Watcher.cpp
  src/CGSolver.cpp
  src/CNSat.cpp
  src/CNSMTGSolver.cpp
  src/Decider.cpp
  src/FormulaTransform.cpp
  src/GSolver.cpp
)

target_include_directories(constraintsolver PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(constraintsolver PUBLIC autodiff alica_engine ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

# catkin_add_gtest(${PROJECT_NAME}-test
#  test/test_constraintsolver.cpp
#  test/test_cnsat.cpp
#  test/test_cnsmtgsolver.cpp
# )
set(THREADS_PREFER_PTHREAD_FLAG ON)
include(CTest)
find_package(Threads REQUIRED)
find_package(GTest REQUIRED)
  include_directories(BEFORE
  ${GTEST_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}-tests
  test/test_constraintsolver.cpp
   test/test_cnsat.cpp
   test/test_cnsmtgsolver.cpp
)

file(COPY test/cnf/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/cnf/)
file(COPY test/etc/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/etc/)


target_link_libraries(${PROJECT_NAME}-tests PRIVATE Threads::Threads ${PROJECT_NAME} alica_engine ${GTEST_LIBRARIES})

add_test(NAME ${PROJECT_NAME}-tests COMMAND ${PROJECT_NAME}-tests)


install(TARGETS ${PROJECT_NAME}
  EXPORT ${CMAKE_PROJECT_NAME}Targets 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
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
