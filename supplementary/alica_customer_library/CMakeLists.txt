cmake_minimum_required(VERSION 3.10)
project(alica_customer_library)

add_compile_options(-std=c++17)

set(PKG_DEPS
  roscpp
  alica_ros_proxy
  constraintsolver
  turtlesim
  geometry_msgs
)

find_package(yaml-cpp REQUIRED)
find_package(alica_engine REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

# ##########
# # Build ##
# ##########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
message(${catkin_INCLUDE_DIRS})

set(alica_customer_library_SOURCES
  src/AcmeBehaviour.cpp
  src/AcmePlan.cpp
  src/AcmeRuntimeCondition.cpp
  src/WaitBehaviour.cpp
)

set(alica_customer_library_HEADER
  include/AcmeBehaviour.h
  include/AcmePlan.h
  include/AcmeRuntimeCondition.h
  include/WaitBehaviour.h
)

add_library(alica_customer_library SHARED
  ${alica_customer_library_SOURCES};
  ${alica_customer_library_HEADER};
)
target_link_libraries(alica_customer_library ${catkin_LIBRARIES})

if(NOT catkin_FOUND)
  target_include_directories(alica_customer_library PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif(NOT catkin_FOUND)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
