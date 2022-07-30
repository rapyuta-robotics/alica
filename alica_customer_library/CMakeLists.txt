cmake_minimum_required(VERSION 3.10)
project(alica_customer_library)

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


add_library(alica_customer_library
    ${sources};
)
target_link_libraries(alica_customer_library ${catkin_LIBRARIES})

if (NOT catkin_FOUND)
 target_include_directories(alica_customer_library PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif (NOT catkin_FOUND)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)