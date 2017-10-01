# RS232 robot firmware

omnidirectional low cost robot firmware

#install:

first, compile the custom msgs package, so the other packages can compile correctly:

catkin_make --pkg tcc_msgs

sudo apt-get install ros-kinetic-realtime-tools

#image_encodings problem

add the line #include <stdlib.h> to /opt/ros/kinetic/include/sensor_msgs/image_encodings.h

then

catkin_make
