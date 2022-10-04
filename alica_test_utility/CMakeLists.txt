cmake_minimum_required(VERSION 3.10.2)
project(alica_test_utility)

add_compile_options(-std=c++17)
set(PKG_DEPS
	
	
)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake) 


find_package(catkin REQUIRED ${PKG_DEPS})
find_package(alica_common_config REQUIRED)
find_package(alica_engine REQUIRED)
find_package(alica_dummy_proxy REQUIRED)
find_package(alica_solver_interface REQUIRED)
# catkin_package(
# 	INCLUDE_DIRS include
# 	LIBRARIES alica_test_utility
# 	CATKIN_DEPENDS ${PKG_DEPS}
# 	DEPENDS alica_common_config alica_engine alica_dummy_proxy alica_solver_interface
# )

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${alica_solver_interface_INCLUDE_DIRS}
	${alica_dummy_proxy_INCLUDE_DIRS}
	${alica_common_config_INCLUDE_DIRS}
	${alica_engine_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/TestBehaviourCreator.cpp
	src/TestContext.cpp
	src/IdleBehaviour.cpp
	src/Util.cpp
)

target_sources(${PROJECT_NAME} PRIVATE
src/TestBehaviourCreator.cpp
	src/TestContext.cpp
	src/IdleBehaviour.cpp
	src/Util.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
)


install(TARGETS ${PROJECT_NAME}
EXPORT ${CMAKE_PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)



export(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake
 )
# exported targets
install(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_PROJECT_NAME}Targets.cmake
  DESTINATION ${ConfigPackageLocation})

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  VERSION 0.0.1
  COMPATIBILITY SameMajorVersion)

  set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})
configure_package_config_file(
  ${CMAKE_PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${ConfigPackageLocation}
  PATH_VARS INCLUDE_INSTALL_DIR)

  install(FILES 
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${ConfigPackageLocation})