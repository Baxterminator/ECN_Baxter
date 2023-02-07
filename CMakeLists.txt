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
file(GLOB msg_files RELATIVE "${CMAKE_CURRENT_LIST_DIR}"
        "${CMAKE_CURRENT_LIST_DIR}/msg/*.msg"
        "${CMAKE_CURRENT_LIST_DIR}/srv/*.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files})
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
ament_auto_add_executable(gripper_node components/gripper_node.cpp)
target_include_directories(gripper_node PRIVATE
        include/ecn_baxter
        include/rapidjson)
add_message(gripper_node)

ament_auto_add_executable(game_master
        game/game_master.cpp
        game/node.cpp
        game/gui_wrapper.cpp
        resource/ui/main.ui
)
target_include_directories(game_master PRIVATE
        include/ecn_baxter
        ${ROS1_ROOT}/include
)
target_link_libraries(game_master ${ROS1_LIBS} Qt5::Core Qt5::Widgets Qt5::WebKit Qt5::WebKitWidgets)

# =================================================================
#                        EXPORT OTHERS
# =================================================================
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_auto_package()
