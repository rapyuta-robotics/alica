cmake_minimum_required(VERSION 3.5.1)
project(autodiff)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")


find_package(catkin REQUIRED COMPONENTS
 fsystem
 alica_engine
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES autodiff alica_engine
 CATKIN_DEPENDS fsystem
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(autodiff
  src/CompiledDifferentiator.cpp
  src/Abs.cpp
  src/And.cpp
  src/Atan2.cpp
  src/Constant.cpp
  src/ConstPower.cpp
  src/ConstraintBuilder.cpp
  src/ConstraintUtility.cpp
  src/Cos.cpp
  src/Exp.cpp
  src/Gp.cpp
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
  src/Term.cpp
  src/TermBuilder.cpp
  src/TermPower.cpp
  src/TermUtils.cpp
  src/TVec.cpp
  src/Variable.cpp
  src/Zero.cpp
  src/compiled/CompiledAbs.cpp
  src/compiled/CompiledAnd.cpp
  src/compiled/CompiledAtan2.cpp
  src/compiled/CompiledConstant.cpp
  src/compiled/CompiledConstPower.cpp
  src/compiled/CompiledConstraintUtility.cpp
  src/compiled/CompiledCos.cpp
  src/compiled/CompiledExp.cpp
  src/compiled/CompiledGp.cpp
  src/compiled/CompiledLinSigmoid.cpp
  src/compiled/CompiledLog.cpp
  src/compiled/CompiledLTConstraint.cpp
  src/compiled/CompiledLTEConstraint.cpp
  src/compiled/CompiledMax.cpp
  src/compiled/CompiledMin.cpp
  src/compiled/CompiledOr.cpp
  src/compiled/CompiledProduct.cpp
  src/compiled/CompiledReification.cpp
  src/compiled/CompiledSigmoid.cpp
  src/compiled/CompiledSin.cpp
  src/compiled/CompiledSum.cpp
  src/compiled/CompiledTermPower.cpp
  src/compiled/CompiledVariable.cpp
)

target_link_libraries(autodiff ${catkin_LIBRARIES})

# AUTODIFF TESTS
find_package(GTest REQUIRED)
include_directories(BEFORE
  ${GTEST_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}-tests test/test_autodiff.cpp)
target_link_libraries(${PROJECT_NAME}-tests ${PROJECT_NAME} pthread ${GTEST_LIBRARIES})

