cmake_minimum_required(VERSION 3.5.1)
project(autodiff)

if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++14" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")


find_package(catkin REQUIRED COMPONENTS
 alica_solver_interface
 alica_common_config
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES autodiff
 CATKIN_DEPENDS alica_solver_interface alica_common_config
)

include_directories(include include/autodiff ${catkin_INCLUDE_DIRS})

add_library(autodiff
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

target_link_libraries(autodiff ${catkin_LIBRARIES})

add_executable(sine_benchmark test/test_benchmark.cpp)

target_link_libraries(sine_benchmark ${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}-test test/test_autodiff.cpp)
    if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
    endif()
else(CATKIN_ENABLE_TESTING)
    # AUTODIFF TESTS
    find_package(GTest REQUIRED)
    include_directories(BEFORE
    ${GTEST_INCLUDE_DIRS}
    )
    add_executable(${PROJECT_NAME}-tests test/test_autodiff.cpp)
    target_link_libraries(${PROJECT_NAME}-tests ${PROJECT_NAME} pthread ${GTEST_LIBRARIES})
endif(CATKIN_ENABLE_TESTING)
