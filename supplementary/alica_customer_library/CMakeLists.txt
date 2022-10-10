cmake_minimum_required(VERSION 3.10)
project(alica_customer_library)

add_compile_options(-std=c++17)

set(PKG_DEPS
  roscpp
  alica_engine
  alica_common_config
  alica_ros_proxy
  constraintsolver
  turtlesim
  geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

# find_package(catkin QUIET)
# if (catkin_FOUND)
# find_package(catkin REQUIRED COMPONENTS
# alica_engine
# alica_common_config
# )
# else (catkin_FOUND)
# set(catkin_LIBRARIES "alica_engine" alica_common_config)
# endif(catkin_FOUND)
if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
    LIBRARIES alica_dummy_proxy
    CATKIN_DEPENDS alica_engine

    # DEPENDS
  )
endif(catkin_FOUND)

# ##########
# # Build ##
# ##########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
message(${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE sources "src/*.cpp")

add_library(alica_customer_library
  ${sources};
)
target_link_libraries(alica_customer_library ${catkin_LIBRARIES})

if(NOT catkin_FOUND)
  target_include_directories(alica_customer_library PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif(NOT catkin_FOUND)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
