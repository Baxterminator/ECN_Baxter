cmake_minimum_required(VERSION 3.5)
project(ecn_baxter VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Qt5 COMPONENTS Core Widgets WebKit WebKitWidgets REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
add_definitions(${EIGEN3_DEFINITIONS})

# =================================================================
#                           ROS 1
# =================================================================
#find ROS 1 packages we depend on
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
#             LEGACY CODE (FOXY, GALACTIC COMPATIBILITY)
# =================================================================
if("$ENV{ROS_DISTRO}" STREQUAL "galactic" OR "$ENV{ROS_DISTRO}" STREQUAL "foxy")
    set(LEGACY_IDL TRUE)
    add_compile_definitions(LEGACY_IDL)
else()
    set(LEGACY_IDL FALSE)
endif()

# =================================================================
#                           INTERFACE
# =================================================================
file(GLOB msg_files RELATIVE "${CMAKE_CURRENT_LIST_DIR}"
        "${CMAKE_CURRENT_LIST_DIR}/msg/*.msg"
        "${CMAKE_CURRENT_LIST_DIR}/srv/*.srv"
        "${CMAKE_CURRENT_LIST_DIR}/action/*.action"
)
set(dependencies
        "geometry_msgs")
rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        DEPENDENCIES ${dependencies})
ament_export_dependencies(rosidl_default_runtime)

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
#                        QT 5 UI Generator
# =================================================================
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS resource/ui)
set(CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_RELEASE} -fprofile-arcs -ftest-coverage")
set_target_properties(Qt5::Core PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "RELEASE")

# Excluding files rosidl files from autouic generation
file(GLOB_RECURSE exclude_file
        "${CMAKE_CURRENT_BINARY_DIR}/ecn*/*.[hc]"
        "${CMAKE_CURRENT_BINARY_DIR}/ecn*/*.[hc]pp"
        "${CMAKE_CURRENT_BINARY_DIR}/ros*/*.[hc]"
        "${CMAKE_CURRENT_BINARY_DIR}/ros*/*.[hc]pp"
        )
foreach(X IN LISTS exclude_file)
    set_property(SOURCE ${X} PROPERTY SKIP_AUTOUIC ON)
endforeach()

# =================================================================
#                              OUTPUT
# =================================================================
############################### Gripper ################################
ament_auto_add_executable(gripper_node src/components/gripper_node.cpp)
target_include_directories(gripper_node PRIVATE include)
add_message(gripper_node)

############################### Jacobian ###############################
ament_auto_add_executable(jacobian src/game/utils/jacobian.cpp)
target_include_directories(jacobian PRIVATE include)
add_message(jacobian)

############################# Setup Server #############################
ament_auto_add_executable(setup_server 
    src/setup/point_server.cpp
    src/setup/setup_server.cpp
)
target_include_directories(setup_server PRIVATE include)
add_message(setup_server)

############################# Game Library #############################
ament_auto_add_library(game_utils
    src/game/data/local_games.cpp
    src/game/data/game_properties.cpp
)
target_include_directories(game_utils PRIVATE include ${ROS1_ROOT}/include)

############################# Game Master #############################
ament_auto_add_executable(game_master
    src/game/events/register_events.cpp
    src/game/utils/logger.cpp
    src/game/game_master.cpp
    src/game/ros1/game_master_1.cpp
    src/game/ros1/bridge_lookup.cpp
    src/game/ros1/tf_broadcast.cpp
    src/game/ros2/game_master_2.cpp
    src/game/properties_loader.cpp
    src/game/ros2/client_points.cpp
    src/game/master/game.cpp
    src/game/ui/main_wrapper.cpp
    src/game/ui/file_loader_wrapper.cpp
    resource/ui/main.ui
    resource/ui/game_loader.ui
)
target_include_directories(game_master PRIVATE include ${ROS1_ROOT}/include)
target_link_libraries(game_master ${game_utils} ${ROS1_LIBS} Qt5::Core Qt5::Widgets Qt5::WebKit Qt5::WebKitWidgets )
add_message(game_master)

# =================================================================
#                           EXPORT OTHERS
# =================================================================
install(DIRECTORY 
        resource/launch
        resource/games
        DESTINATION share/${PROJECT_NAME})

ament_auto_package()