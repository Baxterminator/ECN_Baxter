cmake_minimum_required(VERSION 3.5)
project(ecn_baxter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(baxter_core_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Set custom files
set(msg_files
        "msg/BaxterAction.msg"
        )
file(GLOB_RECURSE RapidJSON include/rapidjson/*.h)

# Messages compilation
rosidl_generate_interfaces(
        ${PROJECT_NAME}
        ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(gripper_node
        include/gripper_node.h src/gripper_node.cpp
        ${RapidJSON})
ament_target_dependencies(
        gripper_node rclcpp baxter_core_msgs
)
target_link_libraries(gripper_node "${cpp_typesupport_target}")
install(TARGETS
        gripper_node
        DESTINATION include/${PROJECT_NAME})

ament_package()
