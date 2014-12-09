# Ros UDP Proxy Generator #

This tool generates a Ros-Proxy Node that serializes predefined topics/messages and transmits it via udp multicast. Vice versa multicast messages are received, deserialized, and send via the original ros topic.
This is particularly useful to apply ros-multi-master solutions.

## Usage ##

* Ensure you have set the 'DOMAIN_CONFIG_FOLDER' to the folder to the location of your 'UdpProxy.conf'. We recommend to add the following line to your '~/.bashrc'

> export DOMAIN_CONFIG_FOLDER="${DOMAIN_FOLDER}/etc"

and copy the UdpProxy.conf to the according folder

* Set a proper MulticastAddress and Port in your 'UdpProxy.conf' to the section '[UdpProxy]'. (You can also stick you the example file provided in this package!)

* Create a new ros package within your catkin workspace:
> catkin_create_pkg PROXYNAME

* Copy the following lines and the end of your 'CMakelists.txt':
```
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
```

* Add to to 'package-xml':
<pre><code>
  &#60;build_depend&#62;system_config&#60;/build_depend&#62;
  &#60;build_depend&#62;roscpp&#60;/build_depend&#62;
  &#60;build_depend&#62;message_generation&#60;/build_depend&#62;
  &#60;build_depend&#62;udp_proxy_generator&#60;/build_depend&#62;
  &#60;build_depend&#62;roslib&#60;/build_depend&#62;

  &#60;run_depend&#62;system_config&#60;/run_depend&#62;
  &#60;run_depend&#62;roscpp&#60;/run_depend&#62;
  &#60;run_depend&#62;message_runtime&#60;/run_depend&#62;
  &#60;run_depend&#62;roslib&#60;/run_depend&#62;
</code></pre>

Plus all the message dependencies you need, e.g., geometry_msgs

* Add a 'relayMsgs.conf' file into your proxy node folder and add the following line for each topic/message you want transmit via multicast:
<pre><code>
 Topic: /topic                    Msg: package/messagename                     Opt:[Udp2RosQueueLength=1 Ros2UdpQueueLength=1]
</code></pre>
The only available options are Udp2RosQueueLength and Ros2UdpQueueLength. Here you can specify the number of message to be queued when received via udp respectivly via ros.


## Requirements ##

You need the system_config package, that is also included in this repository/stack!

