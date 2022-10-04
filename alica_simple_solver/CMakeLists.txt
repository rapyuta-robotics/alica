cmake_minimum_required(VERSION 3.10)
project(alica_simple_solver)

add_compile_options(-std=c++17)

if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++14" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED)
find_package(alica_engine REQUIRED)
find_package(alica_common_config REQUIRED)
find_package(alica_solver_interface REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES alica_simple_solver
  DEPENDS alica_common_config alica_solver_interface alica_engine
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${alica_common_config_INCLUDE_DIRS}
 ${alica_solver_interface_INCLUDE_DIRS}
 ${alica_engine_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
		src/SimpleSolver.cpp
		src/SimpleTerm.cpp
		src/SimpleVariable.cpp
		)

 target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/alica/reasoner
  DESTINATION include/alica
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)