cmake_minimum_required(VERSION 2.8.3)
project(main_planner)

find_package(catkin REQUIRED COMPONENTS  
  roscpp
  std_msgs
  tcc_msgs
  tf
)

catkin_package()

set(CPP_FILES
    src/libs/Planner.cpp
    src/libs/routeInterpreter.cpp
    src/a_star_libs/GraphNode.cpp
    src/a_star_libs/LibAstar.cpp
    src/a_star_libs/Point.cpp
)

set(H_FILES
    include/main_planner/Planner.h
    include/main_planner/routeInterpreter.h
    include/a_star/GraphNode.h
    include/a_star/LibAstar.h
    include/a_star/Point.h
)

include_directories(   
  ${catkin_INCLUDE_DIRS}
)

add_executable(main_planner src/main_planner.cpp ${CPP_FILES})
target_link_libraries(main_planner ${catkin_LIBRARIES})
add_dependencies(main_planner tcc_msgs ${catkin_EXPORTED_TARGETS})
