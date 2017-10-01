# RS232 robot firmware

omnidirectional low cost robot firmware



Dependencies issues that you might have:

sudo apt-get install ros-kinetic-realtime-tools
sudo apt-get install ros-kinetic-tf
sudo apt-get install ros-kinetic-cv-bridge
sudo apt-get install ros-kinetic-qt*
sudo apt-get isntall ros-kinetic-nav-msgs
sudo apt-get install ros-kinetic-visualization-msgs



#Compilation:

first, compile the custom msgs package, so the other packages can compile correctly:

catkin_make --pkg tcc_msgs

#image_encodings problem

add the line #include <stdlib.h> to /opt/ros/kinetic/include/sensor_msgs/image_encodings.h

then

catkin_make
