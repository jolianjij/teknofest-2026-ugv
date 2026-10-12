cmake_minimum_required(VERSION 3.8)
project(ugv_stage_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StageSign.msg"
  "msg/StageTransition.msg"
  "action/ExecuteStage.action"
  DEPENDENCIES action_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
