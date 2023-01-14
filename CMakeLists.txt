cmake_minimum_required(VERSION 3.5)
project(ecn_baxter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(NEW_DISTRO OFF)

macro(addPackage)
    # Did we get any optional args?
    list(LENGTH packages pkg_count)
    if (${pkg_count} GREATER 0)
        foreach(pkg IN LISTS packages)
            find_package(${pkg} REQUIRED)
        endforeach()
    endif ()
endmacro()

macro(addNode node_name dir)
    add_executable("${node_name}"
            "${dir}/${node_name}.h" "${dir}/${node_name}.cpp"
    )
    target_link_libraries("${node_name}" RapidJSON)
    ament_target_dependencies(
            "${node_name}"
            ${packages}
    )
    if (NEW_DISTRO)
        rosidl_get_typesupport_target(cpp_typesupport_target
                ${PROJECT_NAME} "rosidl_typesupport_cpp")
        target_link_libraries(${node_name} "${cpp_typesupport_target}")
    else()
        rosidl_target_interfaces("${node_name}"
                ${PROJECT_NAME} "rosidl_typesupport_cpp")
    endif()

    install(TARGETS
            "${node_name}"
            DESTINATION lib/${PROJECT_NAME})
endmacro()

##############################################################################################
#
#                                  PROJECT CONFIGURATION
#
##############################################################################################

set(packages
        "rclcpp"
        "baxter_core_msgs")
set(msg_files
        "msg/BaxterAction.msg"
        )
file(GLOB_RECURSE RapidJSON_File rapidjson/*.h)
add_library(RapidJSON INTERFACE)
target_include_directories(RapidJSON INTERFACE RapidJSON_File)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
addPackage(${packages})

# Messages compilation
rosidl_generate_interfaces(
        ${PROJECT_NAME}
        ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

addNode(gripper_node "components/gripper" ${packages})
#target_include_directories(gripper_node PRIVATE include)

ament_package()
