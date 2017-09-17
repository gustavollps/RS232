cmake_minimum_required(VERSION 2.8.3)
project(interrupts)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tcc_msgs
)



catkin_package()

FIND_LIBRARY(WIRINGPI_LIBRARY wiringPi /home/odroid/wiringPi)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(interrupts src/interrupts.cpp)
add_dependencies(interrupts tcc_msgs wiringPi ${catkin_EXPORTED_TARGETS})
target_link_libraries(interrupts ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY})
