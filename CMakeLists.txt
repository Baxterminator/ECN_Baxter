cmake_minimum_required(VERSION 3.5)
project(ecn_baxter VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Qt5 COMPONENTS Core Widgets WebKit WebKitWidgets REQUIRED)

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
else()
    set(LEGACY_IDL FALSE)
endif()

function(legacy_cpp files out_l)
    foreach(f ${${files}})
        if(${LEGACY_IDL})
            string(CONCAT _out "src/legacy/" ${f})
        else()
            string(CONCAT _out "src/" ${f})
        endif()
        set(${out_l} ${${out_l}} ${_out})
    endforeach()    
    set(${out_l} ${${out_l}} PARENT_SCOPE)
endfunction(legacy_cpp)


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
## Gripper management
ament_auto_add_executable(gripper_node src/components/gripper_node.cpp)
target_include_directories(gripper_node PRIVATE
        include/ecn_baxter
        include/rapidjson
)
add_message(gripper_node)

## Setup Node
ament_auto_add_executable(setup_node src/game/setup_node.cpp)
target_include_directories(setup_node PRIVATE
        include
)
add_message(setup_node)

## Main Game Master executable
set(legacy_files_gm
    "setup/setup_client.cpp"
)
legacy_cpp(legacy_files_gm legacy_gm)
ament_auto_add_executable(game_master
        src/game/game_master.cpp
        src/game/game_master_1.cpp
        src/game/game_master_2.cpp
        src/setup/client_points.cpp
        src/game/game.cpp
        src/ui/main_wrapper.cpp
        src/ui/file_loader_wrapper.cpp
        resource/ui/main.ui
        resource/ui/game_loader.ui
        ${legacy_gm}
)
target_include_directories(game_master PRIVATE
        include
        ${ROS1_ROOT}/include
)
target_link_libraries(game_master ${ROS1_LIBS} Qt5::Core Qt5::Widgets Qt5::WebKit Qt5::WebKitWidgets)
add_message(game_master)

# =================================================================
#                        EXPORT OTHERS
# =================================================================
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_auto_package()