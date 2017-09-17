cmake_minimum_required(VERSION 2.8.3)
project(heading)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tcc_msgs
)

catkin_package()

FIND_LIBRARY(WIRINGPI_LIBRARY wiringPi /home/odroid/wiringPi)

set(CPP_FILES
    src/libs/DigitalFilter.cpp
    src/libs/mpu.cpp
    src/libs/EKF_IMU.cpp
)

set(H_FILES
    include/heading/DigitalFilter.h
    include/heading/maths.h
    include/heading/mpu.h
    include/heading/EKF_IMU.cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(heading src/heading.cpp ${CPP_FILES})
target_link_libraries(heading ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY})
add_dependencies(heading tcc_msgs wiringPi ${catkin_EXPORTED_TARGETS})

add_executable(heading_ekf src/heading_ekf.cpp ${CPP_FILES})
target_link_libraries(heading_ekf ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY})
add_dependencies(heading_ekf tcc_msgs wiringPi ${catkin_EXPORTED_TARGETS})
