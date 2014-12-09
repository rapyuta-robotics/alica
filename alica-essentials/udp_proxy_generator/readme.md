# Ros UDP Proxy Generator #

This tool generates a Ros-Proxy Node that serializes predefined topics/messages and transmits it via udp multicast. Vice versa multicast messages are received, deserialized, and send via the original ros topic.
This is particularly useful to apply ros-multi-master solutions.

## Usage ##

* Ensure you have set the 'DOMAIN_CONFIG_FOLDER' to the folder to the location of your 'UdpProxy.conf'. We recommend to add the following line to your '~/.bashrc'
> DOMAIN_CONFIG_FOLDER="${DOMAIN_FOLDER}/etc"
and copy the UdpProxy.conf to the according folder

* Set a proper MulticastAddress and Port in your UdpProxy.conf. (You can also stick you the example file provided in this package!)

* Create a new ros package within your catkin workspace:
> catkin_create_pkg PROXYNAME

* Copy the following lines and the end of your 'CMakelists.txt'
<pre><code>
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
add_dependencies(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} msl_sensor_msgs_gencpp)
</code></pre>

* Add to to 'package-xml':
<pre><code>
  <build_depend>system_config</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>udp_proxy_generator</build_depend>
  <build_depend>roslib</build_depend>

  <run_depend>system_config</run_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>message_runtime</run_depend>
  <run_depend>roslib</run_depend>
</code></pre>

Plus all the message dependencies you need, e.g., geometry_msgs

* Add a 'relayMsgs.conf' file into your proxy node folder and add the following line for each topic/message you want transmit via multicast:
<pre><code>
 Topic: /topic                    Msg: package/messagename                     Opt:[Udp2RosQueueLength=1 Ros2UdpQueueLength=1]
</code></pre>
The only available options are Udp2RosQueueLength and Ros2UdpQueueLength. Here you can specify the number of message to be queued when received via udp respectivly via ros.


## Requirements ##

You need the system_config package

