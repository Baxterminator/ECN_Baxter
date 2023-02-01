cmake_minimum_required(VERSION 3.5)
project(ecn_baxter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(msg_files "msg/BaxterAction.msg")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Messages compilation
rosidl_generate_interfaces(
        ${PROJECT_NAME}
        ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

ament_auto_add_executable(gripper_node components/gripper_node.cpp)
target_include_directories(gripper_node PRIVATE include)
rosidl_target_interfaces(gripper_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_auto_add_library(step_node game/step_node.cpp)

ament_package()
