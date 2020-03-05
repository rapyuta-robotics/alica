cmake_minimum_required(VERSION 3.5.1)
project(constraintsolver)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  fsystem
  autodiff
  system_config
  alica_engine
  alica_ros_proxy
  alica_solver_interface
  alica_common_config
  roslib
)

catkin_package(
 # exporting include/constraintsolver is required to avoid breaking existing code
 # TODO: fix existing code to include headers using constraintsolver/<header> & then remove this
 INCLUDE_DIRS include include/constraintsolver
 LIBRARIES constraintsolver
 CATKIN_DEPENDS fsystem autodiff roslib alica_solver_interface alica_common_config
)

include_directories(
  include
  # TODO: see comment in catkin_package() above to remove this
  include/constraintsolver
  ${catkin_INCLUDE_DIRS}
)

add_library(constraintsolver
  src/intervals/DownwardPropagator.cpp
  src/intervals/IntervalPropagator.cpp
  src/intervals/RecursivePropagate.cpp
  src/intervals/ResetIntervals.cpp
  src/intervals/SetParents.cpp
  src/intervals/UnsolveableException.cpp
  src/intervals/UpwardPropagator.cpp
  src/types/Clause.cpp
  src/types/DecisionLevel.cpp
  src/types/Lit.cpp
  src/types/Var.cpp
  src/types/Watcher.cpp
  src/CGSolver.cpp
  src/CNSat.cpp
  src/CNSMTGSolver.cpp
  src/Decider.cpp
  src/FormulaTransform.cpp
  src/GSolver.cpp
)

target_link_libraries(constraintsolver ${catkin_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}-test
 test/test_constraintsolver.cpp
 test/test_cnsat.cpp
 test/test_cnsmtgsolver.cpp
)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

# install headers under constraintsolver/ to global include/ [required to avoid breaking existing code]
# TODO: fix existing code to include headers using constraintsolver/<header> & then remove this
install(DIRECTORY include/constraintsolver/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
