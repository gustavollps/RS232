cmake_minimum_required(VERSION 2.8.3)
project(a_star)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tcc_msgs
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(H_FILES
    include/a_star_test/LibAstar.h
    include/a_star_test/Point.h
    include/a_star_test/GraphNode.h
    include/a_star_test/AStarNode.h
    )

set(CPP_FILES
    src/libs/LibAstar.cpp
    src/libs/Point.cpp
    src/libs/GraphNode.cpp
    src/libs/AStarNode.cpp
    )


add_executable(a_star src/a_star.cpp ${CPP_FILES})
add_dependencies(a_star tcc_msgs ${catkin_EXPORTED_TARGETS})
target_link_libraries(a_star ${catkin_LIBRARIES})
