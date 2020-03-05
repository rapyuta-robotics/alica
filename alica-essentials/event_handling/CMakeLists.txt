cmake_minimum_required(VERSION 3.5.1)
project(event_handling)

add_compile_options(-std=c++14)

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

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
