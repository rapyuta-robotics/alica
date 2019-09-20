cmake_minimum_required(VERSION 2.8.3)
project(alica_capnzero_proxy)

## Use c++ 11x std
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  alica_engine
  capnzero
  id_capnzero
)

catkin_package(
        INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
        LIBRARIES alica_capnzero_proxy alica_msgs
        CATKIN_DEPENDS alica_engine capnzero id_capnzero
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

#### ALICA CAPNZERO MESSAGES
find_package(CapnProto REQUIRED)
set(CAPNPC_IMPORT_DIRS "${id_capnzero_SOURCE_PREFIX}/include")# include external capnp message schema files
set(${PROJECT_NAME}_msgs "${CMAKE_CURRENT_SOURCE_DIR}/include/alica_msg")
file(GLOB_RECURSE capnp_messages ${${PROJECT_NAME}_msgs} *.capnp)
set(CAPNPC_SRC_PREFIX ${${PROJECT_NAME}_msgs})
set(CAPNPC_OUTPUT_DIR ${${PROJECT_NAME}_msgs})
capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS ${capnp_messages})

add_library(alica_msgs ${CAPNP_SRCS})
target_link_libraries(alica_msgs
  ${catkin_LIBRARIES}
)

#### ALICA CAPNZERO PROXY
add_library(${PROJECT_NAME}
    src/Communication.cpp
    src/ContainerUtils.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    alica_msgs
)

#### ALICA CAPNZERO PROXY TESTS
add_executable(senderTest src/SenderTest.cpp)
target_link_libraries(senderTest
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_executable(recieverTest src/RecieverTest.cpp)
target_link_libraries(recieverTest
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)