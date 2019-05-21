cmake_minimum_required(VERSION 2.8.3)
project(alica_capnzero_proxy)

## Use c++ 11x std
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  alica_engine
  capnzero
  id
)

catkin_package(
        INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
        LIBRARIES alica_capnzero_proxy alica_capnz_msgs
        CATKIN_DEPENDS alica_engine capnzero
)

#### ALICA CAPNZERO MESSAGES
find_package(CapnProto REQUIRED)

# include external capnp message schema files
set(CAPNP_INCLUDE_DIRECTORY "${CAPNP_INCLUDE_DIRECTORY} ${id_SOURCE_PREFIX}/include")

#message("ALICACapnZeroProxy: CAPNP_INCLUDE_DIRECTORY is '${CAPNP_INCLUDE_DIRECTORY}'")
#message("ALICACapnZeroProxy: catkin_INCLUDE_DIRS is '${catkin_INCLUDE_DIRS}'")

set(capnzero_eval_message_folder "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB_RECURSE id_msgs ${id_SOURCE_PREFIX}/include/essentials/*.capnp)
file(COPY ${id_msgs} DESTINATION ${capnzero_eval_message_folder})

file(GLOB_RECURSE capnp_messages
        ${capnzero_eval_message_folder} *.capnp)

#file(GLOB_RECURSE capnp_message ${id_SOURCE_PREFIX}/include/essentials *.capnp)
#message("ALICACapnZeroProxy: capnp_messages is '${capnp_messages}'")
set(CAPNPC_SRC_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/msg" CACHE STRING "" FORCE)
file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/alica_capnz_msg")
set(CAPNPC_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/alica_capnz_msg" CACHE STRING "" FORCE)
capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS ${capnp_messages})

add_library(alica_capnz_msgs ${CAPNP_SRCS})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

#### ALICA CAPNZERO PROXY
add_library(${PROJECT_NAME}
    src/communication/AlicaCapnzeroCommunication.cpp
    include/communication/AlicaCapnzeroCommunication.h
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    alica_capnz_msgs
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