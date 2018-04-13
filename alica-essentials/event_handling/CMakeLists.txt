cmake_minimum_required(VERSION 3.5.1)
project(event_handling)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin QUIET)

if(catkin_FOUND)
	catkin_package(
	  INCLUDE_DIRS include
	  LIBRARIES event_handling
	 #CATKIN_DEPENDS other_catkin_pkg
	 #DEPENDS pthread
	)
endif(catkin_FOUND)

include_directories(include)

add_library(event_handling
   src/Timer.cpp
   src/EventTrigger.cpp
   src/Worker.cpp
)

if (NOT catkin_FOUND)
  target_include_directories(event_handling PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif(NOT catkin_FOUND)

# TESTS
if(catkin_FOUND)
    catkin_add_gtest(${PROJECT_NAME}-test test/test_event_handling.cpp)
    if(TARGET ${PROJECT_NAME}-test)
        target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
    endif()
else(catkin_FOUND)
    enable_testing()
    find_package(GTest REQUIRED)
    include_directories(BEFORE
    ${GTEST_INCLUDE_DIRS}
    )
    add_executable(${PROJECT_NAME}-tests test/test_event_handling.cpp)
target_link_libraries(${PROJECT_NAME}-tests ${PROJECT_NAME} pthread ${GTEST_LIBRARIES})
endif(catkin_FOUND)
