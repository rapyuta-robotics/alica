cmake_minimum_required(VERSION 2.8.3)
project(alica_tracing)
add_compile_options(-std=c++17)

set(PKG_DEPS
)

find_package(catkin REQUIRED COMPONENTS alica_engine roscpp ${PKG_DEPS})
find_package(yaml-cpp REQUIRED)

#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_ROOT}/Modules" "/usr/local/lib/cmake/" )
#find_package(jaegertracing CONFIG REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS alica_engine roscpp ${PKG_DEPS}
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM
)

###########
## Build ##
###########

# Declare a C++ library with project namespace to avoid naming collision
add_library(
  ${PROJECT_NAME}
  src/tracing/Trace.cpp
  src/tracing/TraceFactory.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  "-Wl,-rpath=/usr/local/lib/,--disable-new-dtags"
  jaegertracing
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)