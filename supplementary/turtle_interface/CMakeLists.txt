cmake_minimum_required(VERSION 3.5.1)
project(turtle_interface)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Debug)

# Set up utils to allow Colcon/catkin to find the packages
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake)

include(../../cmake_flags/cflags.cmake)
alica_compile_flags()

find_package(alica_engine REQUIRED)
find_package(constraintsolver REQUIRED)
find_package(alica_solver_interface REQUIRED)
find_package(autodiff REQUIRED)


###########
## Build ##
###########

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/turtle_interfaces.cpp
)

set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)


target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  alica_engine
  constraintsolver
  alica_solver_interface
  autodiff
)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  EXPORT ${CMAKE_PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)


# Export targets to make package libraries
export(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake
)

install(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_PROJECT_NAME}Targets.cmake
  DESTINATION ${ConfigPackageLocation}
)

# Installing files to make a find_package()-able package
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  VERSION 1.0.0
  COMPATIBILITY SameMajorVersion
)

set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})

configure_package_config_file(
  ${CMAKE_PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${ConfigPackageLocation}
  PATH_VARS INCLUDE_INSTALL_DIR
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${ConfigPackageLocation}
)


# cmake_minimum_required(VERSION 3.5.1)
# project(turtle_interface)

# include(GNUInstallDirs)
# include(CMakePackageConfigHelpers)
# set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake)

# include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake_flags/cflags.cmake)
# alica_compile_flags()

# find_package(alica_engine REQUIRED)
# find_package(constraintsolver REQUIRED)
# find_package(alica_solver_interface REQUIRED)
# find_package(autodiff REQUIRED)

# include_directories(
#   include
# )

# add_library(${PROJECT_NAME}
#   src/turtle_interfaces.cpp
# )
# target_link_libraries(${PROJECT_NAME} alica_engine constraintsolver alica_solver_interface autodiff)
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )


# install(TARGETS ${PROJECT_NAME}
#   EXPORT ${CMAKE_PROJECT_NAME}Targets
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# install(DIRECTORY include/
#   DESTINATION include
#   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
# )

# #################

# # ament_target_dependencies(turtle_interface
# # constraintsolver alica_solver_interface autodiff
# # )

# # ament_export_include_directories(include)
# # ament_export_libraries(turtle_interface)

# # ament_package()


# export(EXPORT ${CMAKE_PROJECT_NAME}Targets
#   FILE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake
# )

# install(EXPORT ${CMAKE_PROJECT_NAME}Targets
#   FILE ${CMAKE_PROJECT_NAME}Targets.cmake
#   DESTINATION ${ConfigPackageLocation}
# )

# write_basic_package_version_file(
#   ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
#   VERSION 1.0.0
#   COMPATIBILITY SameMajorVersion
# )

# set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})

# configure_package_config_file(
#   ${CMAKE_PROJECT_NAME}Config.cmake.in
#   ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
#   INSTALL_DESTINATION ${ConfigPackageLocation}
#   PATH_VARS INCLUDE_INSTALL_DIR
# )

# install(FILES
#   ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
#   ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
#   DESTINATION ${ConfigPackageLocation}
# )

# # install(DIRECTORY launch etc
# #     DESTINATION share/${PROJECT_NAME}
# # )
