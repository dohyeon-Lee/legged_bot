#ifndef __GROUP_MOTOR_CONTROL_H__
#define __GROUP_MOTOR_CONTROL_H__

#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <vector>
#include "dynamixel_sdk.h"
#include "inverse_kinemetics.h"    
using std::vector;

#define STDIN_FILENO 0
// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

// Data Byte Length
#define LEN_MX_GOAL_POSITION            4
#define LEN_MX_PRESENT_POSITION         2

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_RF0                          1
#define DXL_RF1                          2
#define DXL_RF2                          3

#define DXL_LF0                          9
#define DXL_LF1                          10         
#define DXL_LF2                          11

#define DXL_LB0                          18
#define DXL_LB1                          17
#define DXL_LB2                          15

#define DXL_RB0                          12
#define DXL_RB1                          13
#define DXL_RB2                          14

#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      1023                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

class group_motor_control
{
    public:
        double fmap(double x, double in_min, double in_max, double out_min, double out_max);
        int angle(double angle_);
        void one_leg_moving(vector<double> point, int num); //num is leg number(RF:1 LF:2 LB:3 RB:4)
        void moving(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, dynamixel::GroupSyncWrite groupSyncWrite, vector<vector<double>> point);
        int getch();
        void rest(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, dynamixel::GroupSyncWrite groupSyncWrite);
        void setting(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, dynamixel::GroupSyncWrite groupSyncWrite);
};

#endif