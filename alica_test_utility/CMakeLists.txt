cmake_minimum_required(VERSION 3.10.2)
project(alica_test_utility)

add_compile_options(-std=c++17)
set(PKG_DEPS
	alica_engine
	alica_dummy_proxy
)
find_package(catkin REQUIRED ${PKG_DEPS})

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES alica_test_utility
	CATKIN_DEPENDS ${PKG_DEPS}
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/TestBehaviourCreator.cpp
	src/TestContext.cpp
	src/IdleBehaviour.cpp
	src/Util.cpp
)

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
