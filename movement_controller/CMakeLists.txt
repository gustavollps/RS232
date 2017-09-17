cmake_minimum_required(VERSION 2.8.3)
project(movement_controller)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tcc_msgs
  control_loop
)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES PID
    CATKIN_DEPENDS roscpp std_msgs tcc_msgs control_loop
    )


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PID_INCLUDE_DIRS}
)

set(CPP_FILES
    src/libs/movementController.cpp
    )

add_executable(movement_controller src/movement_controller.cpp ${CPP_FILES})
add_dependencies(movement_controller tcc_msgs PID ${catkin_EXPORTED_TARGETS})
target_link_libraries(movement_controller ${catkin_LIBRARIES} ${PID_LIBRARIES})
