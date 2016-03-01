cmake_minimum_required(VERSION 2.8.3)
project(cling_wrap)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake CACHE INTERNAL "" FORCE)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -fPIC -Wall ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

execute_process(COMMAND uname -m OUTPUT_VARIABLE ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED cmake_modules)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include libgringo libgringo/src libprogram_opts libclasp
  LIBRARIES libcling_wrap libgringo libprogram_opts libclasp
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(include libgringo libgringo/src libprogram_opts libclasp)

### BUILDING GRINGO ###
if(NOT "${ARCH}" STREQUAL "armv7l")
  FIND_PACKAGE(RE2C REQUIRED 0.13.5)
  set(FileToCheck "libgringo/src/input/nongroundlexer.hh")
  ADD_CUSTOM_TARGET(CheckNonGroundLexer ALL
   COMMAND ${CMAKE_COMMAND}
     -DFileToCheck=${FileToCheck}
     -DWORKING_DIRECTORY=${CMAKE_CURRENT_SOURCE_DIR}
     -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenRE2C.cmake
   COMMENT "Checking if ${FileToCheck} exists..."
  )
 
  FIND_PACKAGE(BISON REQUIRED 3.0) 
  BISON_TARGET(GringoParser 
	libgringo/src/input/nongroundgrammar.yy
	${CMAKE_CURRENT_SOURCE_DIR}/libgringo/src/input/nongroundgrammar/grammar.cc
	COMPILE_FLAGS "-r all --report-file=libgringo/src/input/nongroundgrammar/grammar.out"
  )

 file(GLOB_RECURSE libgringo_SOURCES "libgringo/src/*.cc")
 file(GLOB_RECURSE libgringo_SOURCES ${libgringo_SOURCES} "libgringo/src/*.cpp")
 add_library(
  libgringo STATIC ${libgringo_SOURCES} ${BISON_GringoParser_OUTPUTS}
 )
 add_dependencies(libgringo CheckNonGroundLexer)
else(NOT ${ARCH} STREQUAL "armv7l")
 message("We are on an arm-architecture, so we cannot build the ASP Solver Stuff, but that is fine...")
endif(NOT "${ARCH}" STREQUAL "armv7l")

### BUIDLING LIBOPTIONS ###
if(NOT ${ARCH} STREQUAL "armv7l")
  file(GLOB_RECURSE libprogram_opts_SOURCES "libprogram_opts/src/*.cpp")
  add_library(
    libprogram_opts STATIC ${libprogram_opts_SOURCES}
  )
endif(NOT ${ARCH} STREQUAL "armv7l")

### BUIDLING LIBCLASP ###
if(NOT ${ARCH} STREQUAL "armv7l")
  add_definitions(-DWITH_THREADS=1)
  find_package(TBB REQUIRED)
  file(GLOB_RECURSE libclasp_SOURCES "libclasp/src/*.cpp")
  add_library(
    libclasp STATIC ${libclasp_SOURCES}
  )
  
  target_link_libraries(libclasp LINK_PUBLIC ${TBB_LIBRARIES})
endif(NOT ${ARCH} STREQUAL "armv7l")

### BUIDLING CLINGO ###
#  file(GLOB_RECURSE clingo_SOURCES "app/clingo/src/*.cc")
#  add_executable(
#    clingo ${clingo_SOURCES}
#  )
#  target_link_libraries(clingo libgringo libprogram_opts libclasp)

### BUIDLING LIBCLING_WRAP ###
if(NOT ${ARCH} STREQUAL "armv7l")
  file(GLOB_RECURSE libcling_wrap_SOURCES "libcling_wrap/*.cc")
  file(GLOB_RECURSE libcling_wrap_SOURCES ${libcling_wrap_SOURCES} "libcling_wrap/*.cpp")
  file(GLOB_RECURSE libcling_wrap_SOURCES ${libcling_wrap_SOURCES} "app/clingo/src/*.cc")
  add_library(
    libcling_wrap STATIC ${libcling_wrap_SOURCES}
  )
  
  target_link_libraries(libcling_wrap libgringo libprogram_opts libclasp ${TBB_LIBRARIES})
endif(NOT ${ARCH} STREQUAL "armv7l")

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(NOT ${ARCH} STREQUAL "armv7l")
 catkin_add_gtest(${PROJECT_NAME}-test 
   test/src/test_cling_wrap.cpp
   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
 if(TARGET ${PROJECT_NAME}-test)
   target_link_libraries(${PROJECT_NAME}-test libcling_wrap)
 endif()
endif(NOT ${ARCH} STREQUAL "armv7l")
