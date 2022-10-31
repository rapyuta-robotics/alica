cmake_minimum_required(VERSION 3.5.1)
project(constraintsolver)
include(CTest)

add_compile_options(-std=c++17)


include(../../cmake_flags/cflags.cmake)
alica_compile_flags()


# Set up utils to allow Colcon/catkin to find the packages
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake)


##Yaml
find_package(yaml-cpp REQUIRED)


find_package(alica_solver_interface REQUIRED)
find_package(autodiff REQUIRED)
find_package(alica_engine REQUIRED)


include_directories(
  include
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

target_link_libraries(constraintsolver PUBLIC autodiff alica_engine ${YAML_CPP_LIBRARIES})

if(BUILD_TESTING)

set(THREADS_PREFER_PTHREAD_FLAG ON)

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

add_test(NAME AutoDiffTest.GSOLVER COMMAND ${PROJECT_NAME}-tests --gtest_filter=AutoDiffTest.GSOLVER )
add_test(NAME AutoDiffTest.GSOLVER_UTIL COMMAND ${PROJECT_NAME}-tests --gtest_filter=AutoDiffTest.GSOLVER_UTIL )
add_test(NAME CNSatTest.CNSAT0 COMMAND ${PROJECT_NAME}-tests --gtest_filter=CNSatTest.CNSAT0 )
add_test(NAME CNSatTest.CNSATaim_50_1_6_yes1_4 COMMAND ${PROJECT_NAME}-tests --gtest_filter=CNSatTest.CNSATaim_50_1_6_yes1_4 )
add_test(NAME CNSatTest.CNSATpar8_1_c_cnf COMMAND ${PROJECT_NAME}-tests --gtest_filter=CNSatTest.CNSATpar8_1_c_cnf )
add_test(NAME CNSatTest.CNSAT1_aim_100_1_6_no_1cnf COMMAND ${PROJECT_NAME}-tests --gtest_filter=CNSatTest.CNSAT1_aim_100_1_6_no_1cnf )
add_test(NAME CNSatTest.CNSAT1dubois22 COMMAND ${PROJECT_NAME}-tests --gtest_filter=CNSatTest.CNSAT1dubois22 )
add_test(NAME CNSatTest.CNSAThole6 COMMAND ${PROJECT_NAME}-tests --gtest_filter=CNSatTest.CNSAThole6 )
add_test(NAME CNSatTest.CNSAT1_dubois20 COMMAND ${PROJECT_NAME}-tests --gtest_filter=CNSatTest.CNSAT1_dubois20 )

endif()

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


#Export targets to make package libraries
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
