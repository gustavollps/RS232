cmake_minimum_required(VERSION 2.8.3)
project(tcc_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  cmd_vel_msg.msg
  interrupt_counter.msg
  point.msg
  location_goal.msg
  pose2D.msg
  movement_msg.msg
  magdata.msg
)

add_service_files(
  FILES
  changePID.srv
  CalibrateInt.srv
  ChangeGoal.srv
  ZeroAngle.srv
  ResetOdom.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
