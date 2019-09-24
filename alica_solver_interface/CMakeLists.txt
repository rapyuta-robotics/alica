cmake_minimum_required(VERSION 2.8.3)
project(alica_solver_interface)

## Change to use clang ... unfortunately it doesnt work :(
#set(CMAKE_CXX_COMPILER "clang")
#
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++14" CACHE STRING "C++ version for eclipse" FORCE)
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


if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}-test
    test/test_intervals.cpp
    test/test_main.cpp
    )
    if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
    endif()
else(CATKIN_ENABLE_TESTING)
    find_package(GTest REQUIRED)
    include_directories(BEFORE
    ${GTEST_INCLUDE_DIRS}
    )
    add_executable(${PROJECT_NAME}-test
    test/test_intervals.cpp
    test/test_main.cpp
    )
    target_link_libraries(${PROJECT_NAME}-tests ${PROJECT_NAME} pthread ${GTEST_LIBRARIES})
endif(CATKIN_ENABLE_TESTING)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)