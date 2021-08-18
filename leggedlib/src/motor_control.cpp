#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <vector>
#include "motor_control.h"
#include "dynamixel_sdk.h"
#include "inverse_kinemetics.h"    

double motor_control::fmap(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min)*(out_max - out_min) / (in_max - in_min) + out_min;
}

int motor_control::angle(double angle_)
{
    return fmap(angle_, -M_PI/2, M_PI/2, 205, 819);
}

void motor_control::one_leg_moving(vector<double> point, int num)
{
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    portHandler->openPort();
    portHandler->setBaudRate(BAUDRATE);

    IK ik;
    int index = 0;
    vector<double> leg;

    uint8_t dxl_error = 0;                          // Dynamixel error
    uint16_t dxl_present_position = 0;              // Present position
    if (num == 1)
    {
        leg = ik.IK_RF(point[0], point[1], point[2]);
        int position[3] = {angle(leg[0]),angle(-leg[1]),angle(-leg[2])};         // Goal position
        packetHandler->write1ByteTxRx(portHandler, DXL_RF0, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL_RF1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL_RF2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

        
        packetHandler->write2ByteTxRx(portHandler, DXL_RF0, ADDR_MX_GOAL_POSITION, position[0], &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL_RF1, ADDR_MX_GOAL_POSITION, position[1], &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL_RF2, ADDR_MX_GOAL_POSITION, position[2], &dxl_error);
        

        //packetHandler->write1ByteTxRx(portHandler, DXL_RF0, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        //packetHandler->write1ByteTxRx(portHandler, DXL_RF1, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        //packetHandler->write1ByteTxRx(portHandler, DXL_RF2, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        // Close port
        //portHandler->closePort();
    }
    else if(num == 2)
    {
        leg = ik.IK_LF(point[0], point[1], point[2]);
        int position[3] = {angle(leg[0]),angle(-leg[1]),angle(-leg[2])};         // Goal position
        packetHandler->write1ByteTxRx(portHandler, DXL_LF0, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL_LF1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL_LF2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

        
        packetHandler->write2ByteTxRx(portHandler, DXL_LF0, ADDR_MX_GOAL_POSITION, position[0], &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL_LF1, ADDR_MX_GOAL_POSITION, position[1], &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL_LF2, ADDR_MX_GOAL_POSITION, position[2], &dxl_error);

        //packetHandler->write1ByteTxRx(portHandler, DXL_LF0, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        //packetHandler->write1ByteTxRx(portHandler, DXL_LF1, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        //packetHandler->write1ByteTxRx(portHandler, DXL_LF2, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        // Close port
        //portHandler->closePort();
    }
    else if(num == 3)
    {
        leg = ik.IK_LB(point[0], point[1], point[2]);
        int position[3] = {angle(leg[0]),angle(-leg[1]),angle(-leg[2])};         // Goal position
        packetHandler->write1ByteTxRx(portHandler, DXL_LB0, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL_LB1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL_LB2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);


        packetHandler->write2ByteTxRx(portHandler, DXL_LB0, ADDR_MX_GOAL_POSITION, position[0], &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL_LB1, ADDR_MX_GOAL_POSITION, position[1], &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL_LB2, ADDR_MX_GOAL_POSITION, position[2], &dxl_error);
        

        //packetHandler->write1ByteTxRx(portHandler, DXL_LB0, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        //packetHandler->write1ByteTxRx(portHandler, DXL_LB1, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        //packetHandler->write1ByteTxRx(portHandler, DXL_LB2, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        // Close port
       // portHandler->closePort();
    }
    else if(num == 4)
    {
        leg = ik.IK_RB(point[0], point[1], point[2]);
        int position[3] = {angle(leg[0]),angle(-leg[1]),angle(-leg[2])};         // Goal position
        packetHandler->write1ByteTxRx(portHandler, DXL_RB0, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL_RB1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL_RB2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

        
        packetHandler->write2ByteTxRx(portHandler, DXL_RB0, ADDR_MX_GOAL_POSITION, position[0], &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL_RB1, ADDR_MX_GOAL_POSITION, position[1], &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL_RB2, ADDR_MX_GOAL_POSITION, position[2], &dxl_error);
        

        //packetHandler->write1ByteTxRx(portHandler, DXL_RB0, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        //packetHandler->write1ByteTxRx(portHandler, DXL_RB1, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        //packetHandler->write1ByteTxRx(portHandler, DXL_RB2, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        // Close port
        //portHandler->closePort();
    }

}

void motor_control::moving(vector<vector<double>> point)
{
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    portHandler->openPort();
    portHandler->setBaudRate(BAUDRATE);

    IK ik;
    int index = 0;

    uint8_t dxl_error = 0;                          // Dynamixel error
    uint16_t dxl_present_position = 0;              // Present position

    vector<double> leg1 = ik.IK_RF(point[0][0], point[0][1], point[0][2]);
    vector<double> leg2 = ik.IK_LF(point[1][0], point[1][1], point[1][2]);
    vector<double> leg3 = ik.IK_LB(point[2][0], point[2][1], point[2][2]);
    vector<double> leg4 = ik.IK_RB(point[3][0], point[3][1], point[3][2]);

    int position1[3] = {angle(leg1[0]),angle(-leg1[1]),angle(-leg1[2])};
    int position2[3] = {angle(leg2[0]),angle(-leg2[1]),angle(-leg2[2])};
    int position3[3] = {angle(leg3[0]),angle(-leg3[1]),angle(-leg3[2])};
    int position4[3] = {angle(leg4[0]),angle(-leg4[1]),angle(-leg4[2])};

    packetHandler->write1ByteTxRx(portHandler, DXL_RF0, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_RF1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_RF2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_LF0, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_LF1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_LF2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_LB0, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_LB1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_LB2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_RB0, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_RB1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_RB2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

    while(1)
    {
        packetHandler->write2ByteTxRx(portHandler, DXL_RF0, ADDR_MX_GOAL_POSITION, position1[0], &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL_RF1, ADDR_MX_GOAL_POSITION, position1[1], &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL_RF2, ADDR_MX_GOAL_POSITION, position1[2], &dxl_error);

        packetHandler->write2ByteTxRx(portHandler, DXL_LF0, ADDR_MX_GOAL_POSITION, position2[0], &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL_LF1, ADDR_MX_GOAL_POSITION, position2[1], &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL_LF2, ADDR_MX_GOAL_POSITION, position2[2], &dxl_error);

        packetHandler->write2ByteTxRx(portHandler, DXL_LB0, ADDR_MX_GOAL_POSITION, position3[0], &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL_LB1, ADDR_MX_GOAL_POSITION, position3[1], &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL_LB2, ADDR_MX_GOAL_POSITION, position3[2], &dxl_error);

        packetHandler->write2ByteTxRx(portHandler, DXL_RB0, ADDR_MX_GOAL_POSITION, position4[0], &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL_RB1, ADDR_MX_GOAL_POSITION, position4[1], &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL_RB2, ADDR_MX_GOAL_POSITION, position4[2], &dxl_error);
    }

}
