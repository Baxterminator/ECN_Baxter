cmake_minimum_required(VERSION 3.5)
project(ecn_baxter)

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

# =================================================================
#                           OUTPUT
# =================================================================
ament_auto_add_executable(gripper_node components/gripper_node.cpp)
target_include_directories(gripper_node PRIVATE include)
rosidl_target_interfaces(gripper_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_auto_add_executable(game_master game/game_master.cpp game/node.cpp)
target_include_directories(game_master PRIVATE
        include
        ${ROS1_ROOT}/include
        )
target_link_libraries(game_master ${ROS1_LIBS})

# =================================================================
#                          EXPORT HPP
# =================================================================
#include_directories(include/ecn_baxter/export)
#set(HEADER_FILES include/ecn_baxter/export/step_node.hpp)
#add_library(game_lib ${HEADER_FILES})
#set_target_properties(game_lib PROPERTIES LINKER_LANGUAGE CXX)
#ament_export_targets(game_lib HAS_LIBRARY_TARGET)
install(
        DIRECTORY include/${PROJECT_NAME}
        DESTINATION include/${PROJECT_NAME}
)
#install(
#        TARGETS game_lib
#        EXPORT game_lib
#        LIBRARY DESTINATION lib
#        ARCHIVE DESTINATION lib
#        RUNTIME DESTINATION bin
#        INCLUDES DESTINATION include
#)


ament_auto_package()
