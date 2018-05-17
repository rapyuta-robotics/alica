cmake_minimum_required(VERSION 2.8.3)
project(alica_solver_interface)

## Change to use clang ... unfortunately it doesnt work :(
#set(CMAKE_CXX_COMPILER "clang")
#
## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11  ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")


find_package(catkin QUIET)
if (catkin_FOUND)
  find_package(catkin REQUIRED COMPONENTS alica_common_config)
else (catkin_FOUND)
  set(catkin_LIBRARIES "alica_common_config")
endif(catkin_FOUND)



if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES alica_solver_interface
  )
endif(catkin_FOUND)

###########
## Build ##
###########

include_directories(include include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE sources "src/*.cpp")


add_library(alica_solver_interface
    ${sources};
)
target_link_libraries(alica_solver_interface ${catkin_LIBRARIES})

if (NOT catkin_FOUND)
 target_include_directories(alica_solver_interface PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include") 
endif (NOT catkin_FOUND)
