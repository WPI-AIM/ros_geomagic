cmake_minimum_required(VERSION 2.8.3)
project(geomagic_control)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs genmsg rosconsole tf urdf)

add_message_files(FILES DeviceButtonEvent.msg DeviceFeedback.msg)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(CATKIN_DEPENDS roscpp std_msgs geometry_msgs genmsg rosconsole tf urdf)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(device_node src/device_node.cpp)

add_dependencies(device_node geomagic_control_generate_messages_cpp)

target_link_libraries(device_node HD HL HDU rt ncurses ${catkin_LIBRARIES})


