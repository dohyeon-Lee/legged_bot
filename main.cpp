#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <vector>
#include "dynamixel_sdk.h"                                 
#include "motor_control.h"
#include "inverse_kinemetics.h"
#include "group_motor_control.h"
#include "action.h"
#include <unistd.h>

#include <string>
#include <stdexcept>
// C library headers
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <cstdio>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <cstdlib>
#include <csignal>
#include <sys/file.h>
#include "SerialClass.h"
using std::vector;
using std::string;
using namespace std;
int serial_port;
void signalHandler( int signum ) 
{
  close(serial_port);
  exit(signum);  
}

int getch()
{
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

int main(int argc, char *argv[])
{
  //serial setting
  signal(SIGINT, signalHandler);  
  Serial serial(&serial_port, "/dev/ttyACM0",9600);

  //dynamixel setting
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
  portHandler->openPort();
  portHandler->setBaudRate(BAUDRATE);

  group_motor_control legged_bot;
  vector<double> normal = {0,0.3,1}; //for plane & groundslope
  IK body; 
  action act;

  int data = 0;
  double l = 0.15;
  vector<vector<double>> point;
  legged_bot.setting(portHandler, packetHandler, groupSyncWrite);
  int sleep_time = 10000;
  
  //parameters for walking
  /*double t1 = 0;
  double t2 = 0;
  double t3 = 0;
  double t4 = 0;
  */

  if (getch() == ESC_ASCII_VALUE)
  {
    legged_bot.rest(portHandler, packetHandler, groupSyncWrite);
    return 0; 
  }
  double anglex;
  double angley;
  double angle_x;
  double angle_y;

  //PID control
  double pre_P_x = 0;
  double pre_P_y = 0;

  double error_x = 0;
  double error_y = 0;

  while(1)
  {
    //about serial
    serial.readangles(serial_port, &anglex, &angley);
    if(anglex != -1 && angley != -1 && anglex <= 180 && anglex >= -180 && angley <= 180 && angley >= -180)
    {
      angle_x = anglex;
      angle_y = angley;
      //printf("%lf %lf\n", anglex, angley);
    }
    vector<double> goal = {0,0};
    
    error_x = -(goal[0] - angle_x); //minus is because of sensors hardware
    error_y = -(goal[1] - angle_y);
    double Kpx = 0.01;
    double Kpy = 0.01;
    double P_x = pre_P_x + Kpx * error_x;
    double P_y = pre_P_y + Kpy * error_y;

    pre_P_x = P_x;
    pre_P_y = P_y;

    vector<double> angle = body.aaa(P_x, P_y);
    point = body.groundslope(angle,l);
    legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
    usleep(sleep_time);
  }
  return 0;
}

/*point = body.plane(normal,l);
      legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
*/