cmake_minimum_required(VERSION 3.10.2)
project(alica_test_support)

add_compile_options(-std=c++17)
set(PKG_DEPS
	alica_engine
	alica_dummy_proxy
)
find_package(catkin REQUIRED ${PKG_DEPS})

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES alica_test_support
	CATKIN_DEPENDS ${PKG_DEPS}
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/TestBehaviourCreator.cpp
	src/AlicaTestSupportUtility.cpp
    src/mockups/FailBehaviour.cpp
    src/mockups/IdleBehaviour.cpp
    src/mockups/SuccessBehaviour.cpp
    src/mockups/IMockUpBehaviour.cpp
)

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
)
