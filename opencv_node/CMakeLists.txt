cmake_minimum_required(VERSION 2.8.3)
project(opencv_node)
 
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  tcc_msgs
  message_generation
)
 
find_package(OpenCV REQUIRED)

add_service_files(
  FILES
  SetParams.srv
) 

add_message_files(
    FILES
    color.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package() 

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
 
add_executable(opencv_node src/opencv_node.cpp src/CamVision.cpp)
target_link_libraries(opencv_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(opencv_node tcc_msgs opencv_node_generate_messages_cpp)
