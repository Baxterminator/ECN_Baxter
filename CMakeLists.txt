cmake_minimum_required(VERSION 3.5)
project(ecn_baxter VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# =================================================================
#                           ROS 1
# =================================================================
# find ROS 1 packages we depend on
set(ROS1_ROOT "/opt/ros/noetic")
set(ROS1_LIBS roscpp rosconsole roscpp_serialization rostime xmlrpcpp)

# if libraries in ROS1_ROOT, explicitely give their location
if(EXISTS ${ROS1_ROOT})
    foreach(ROS1_LIB ${ROS1_LIBS})
        add_library(${ROS1_LIB} UNKNOWN IMPORTED)
        set_property(TARGET ${ROS1_LIB} PROPERTY IMPORTED_LOCATION "${ROS1_ROOT}/lib/lib${ROS1_LIB}.so")
    endforeach()
endif()

# =================================================================
#                           INTERFACE
# =================================================================
set(msg_files "msg/BaxterAction.msg")
rosidl_generate_interfaces(
        ${PROJECT_NAME}
        ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

if("$ENV{ROS_DISTRO}" STREQUAL "galactic" OR "$ENV{ROS_DISTRO}" STREQUAL "foxy")
    set(LEGACY_IDL TRUE)
else()
    set(LEGACY_IDL FALSE)
endif()

if(NOT ${LEGACY_IDL})
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
endif()

macro(add_message node_name)
    if(${LEGACY_IDL})
        rosidl_target_interfaces(${node_name} "${PROJECT_NAME}" "rosidl_typesupport_cpp")
    else()
        target_link_libraries(${node_name} "${cpp_typesupport_target}")
    endif()
endmacro()

# =================================================================
#                              OUTPUT
# =================================================================
ament_auto_add_executable(gripper_node components/gripper_node.cpp)
target_include_directories(gripper_node PRIVATE include)
add_message(gripper_node)

ament_auto_add_executable(game_master game/game_master.cpp game/node.cpp)
target_include_directories(game_master PRIVATE
        include
        ${ROS1_ROOT}/include
        )
target_link_libraries(game_master ${ROS1_LIBS})

install(PROGRAMS
        gui/ui_node.py
        DESTINATION lib/${PROJECT_NAME}
        )

# =================================================================
#                          EXPORT HPP
# =================================================================
install(
        DIRECTORY include/${PROJECT_NAME}
        DESTINATION include/${PROJECT_NAME}
)

ament_auto_package()
