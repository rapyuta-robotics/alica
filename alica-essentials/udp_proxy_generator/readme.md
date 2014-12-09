# Ros UDP Proxy Generator #

This tool generates a Ros-Proxy Node that serializes predefined topics/messages and transmits it via udp multicast. Vice versa multicast messages are received, deserialized, and send via the original ros topic.
This is particularly useful to apply ros-multi-master solutions.

## Requirements ##

You need the [system_config](https://github.com/carpe-noctem-cassel/supplementary/tree/master/system_config) package, that is also included in this repository/stack! It is necessary to read the multicast address and port from the UdpProxy.conf file.

## Usage ##

* Ensure you have set the 'DOMAIN_CONFIG_FOLDER' to the folder with your 'UdpProxy.conf'. We recommend to add the following line to your '~/.bashrc' and copy the UdpProxy.conf to the folder accordingly.
```    
export DOMAIN_CONFIG_FOLDER="insert_path_to_UdpProxy.conf"
```
* Set a proper MulticastAddress and Port in your 'UdpProxy.conf' to the section '[UdpProxy]'. (You can also stick to the example file provided in this package!)

* Create a new ros package within your catkin workspace:
```
catkin_create_pkg PROXYNAME
```

* Copy the following lines into 'CMakelists.txt':
```
## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED system_config roscpp roslib PACKAGES_WITH_YOUR_MESSAGES)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem regex)


include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Triggers the udp_proxy_generator to create the serialization source
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/proxy_gen/ros2udpProxy.cpp
    COMMAND rosrun udp_proxy_generator udp_proxy_generator ${PROJECT_NAME}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/relayMsgs.conf udp_proxy_generator
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "${PROJECT_NAME}: Generating Code ..."
 )

## Declare a cpp executable
add_executable(${PROJECT_NAME}
  proxy_gen/ros2udpProxy.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} PACKAGES_WITH_YOUR_MESSAGES_gencpp)
```
Replace 'PACKAGES_WITH_YOUR_MESSAGES' with the packages that include the messages you want to serialize

* Add the dependencies to 'package.xml':
```
  <build_depend>system_config</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>udp_proxy_generator</build_depend>
  <build_depend>roslib</build_depend>

  <run_depend>system_config</run_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>message_runtime</run_depend>
  <run_depend>roslib</run_depend>
```

Plus all the message dependencies you need, e.g., geometry_msgs

* Add a 'relayMsgs.conf' file into your proxy node folder and add the following line for each topic/message you want transmit via multicast:
```
 Topic: /topic                    Msg: package/messagename                     Opt:[Udp2RosQueueLength=1 Ros2UdpQueueLength=1]
```
The only available options are Udp2RosQueueLength and Ros2UdpQueueLength. Here you can specify the number of message to be queued when received via udp respectivly via ros.
