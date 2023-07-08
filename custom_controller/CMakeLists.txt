cmake_minimum_required(VERSION 3.0.2)
project(custom_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++11)
add_compile_options(-o2)
add_compile_options(-march=native)
add_compile_options(-lpthread)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2_ros
    )

catkin_package()

include_directories(
    include
    tf2_ros
    ${catkin_INCLUDE_DIRS}
)


## Library Filescatkin_package
set(serial_SRCS
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h )

list(APPEND serial_SRCS src/impl/unix.cc)
list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)

## Build Serial library
add_library(${PROJECT_NAME} ${serial_SRCS})

## Include windows API
target_link_libraries(${PROJECT_NAME} rt pthread)

## Source
set(SRCS
        src/Packet/Packet.cpp
        src/Packet/HostPacket.cpp
        src/Packet/MemoryStream.cpp
        src/Crc.cpp
        src/TransmitterJS.cpp
        src/CustomController.cpp)

add_executable(t265_control_node t265_control_node.cpp ${SRCS})
add_dependencies(t265_control_node ${PROJECT_NAME})
target_link_libraries(t265_control_node ${PROJECT_NAME} ${catkin_LIBRARIES})
