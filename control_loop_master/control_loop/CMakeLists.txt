cmake_minimum_required(VERSION 2.8.3)
project(control_loop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs  
  tcc_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES PID
    CATKIN_DEPENDS roscpp std_msgs tcc_msgs
    )

set(CPP_FILES
    src/libs/ControlLoop.cpp
    src/libs/PID.cpp
)

set(H_FILES
    include/control_loop/ControlLoop.h
    include/control_loop/PID.h
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(control_loop src/control_loop.cpp ${CPP_FILES})
add_dependencies(control_loop tcc_msgs ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_loop ${catkin_LIBRARIES})


add_library(PID src/libs/PID.cpp)
add_dependencies(PID ${catkin_EXPORTED_TARGETS})
target_link_libraries(PID ${catkin_LIBRARIES})
