cmake_minimum_required(VERSION 2.8.3)
project(simulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tcc_msgs
)

catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(H_FILES
    include/simulator/simulator.h

    )

set(CPP_FILES
    src/sim.cpp
    src/libs/simulator.cpp
    )

add_executable(${PROJECT_NAME}_node  src/sim.cpp ${CPP_FILES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

