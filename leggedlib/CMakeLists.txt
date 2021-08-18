cmake_minimum_required (VERSION 2.8)
project (leggedlib)
add_library(leggedlib src/group_bulk_read.cpp src/group_bulk_write.cpp src/group_sync_read.cpp src/group_sync_write.cpp src/packet_handler.cpp src/port_handler.cpp src/protocol1_packet_handler.cpp src/protocol2_packet_handler.cpp src/port_handler_linux.cpp src/inverse_kinemetics.cpp src/motor_control.cpp src/group_motor_control.cpp src/action.cpp)
target_include_directories(leggedlib PUBLIC include)