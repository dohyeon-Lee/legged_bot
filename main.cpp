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

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <cstdlib>
#include <csignal>
#include <sys/file.h>
#include "SerialClass.h"
int serial_port;

void signalHandler( int signum ) {
   // cleanup and close up stuff here  
   // terminate program
    close(serial_port);
    exit(signum);  
}

int main(int argc, char *argv[])
{
    signal(SIGINT, signalHandler);  
    Serial serial(&serial_port, "/dev/ttyACM0",9600);
    
    while (true) 
    {
        char chr;
        int num_bytes;
        do {
            num_bytes = read(serial_port, &chr, 1);
            if (num_bytes < 0) {
                printf("Error reading: %s", strerror(errno));
                break;
            }
            if (num_bytes == 1) {
                printf("%c", chr);

            }
        }
        while (num_bytes);
        nanosleep((const struct timespec[]){{0, 10000000L}}, NULL);
    }

    return 0;
}
/*
using std::vector;

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

int main()
{
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
  portHandler->openPort();
  portHandler->setBaudRate(BAUDRATE);

  group_motor_control legged_bot;
  vector<double> normal = {0,0,1};
  IK body; 
  action act;

  int data = 0;
  double l = 0.15;
  vector<vector<double>> point;
  legged_bot.setting(portHandler, packetHandler, groupSyncWrite);
  int sleep_time = 10000;
  double t1 = 0;
  double t2 = 0;
  double t3 = 0;
  double t4 = 0;

  if (getch() == ESC_ASCII_VALUE)
  {
    legged_bot.rest(portHandler, packetHandler, groupSyncWrite);
    return 0;
  }
  while(1)
  {
    t1 = t1 + 0.00003;
    t2 = t2 + 0.00003;
    t3 = t3 + 0.00003;
    t4 = t4 + 0.00003;
    point = act.forward(&t1, &t2, &t3, &t4);
    legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
    usleep(sleep_time);
  }
  
  

  return 0;
}
*/
/*
printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

  int count = 0;
    if(std::abs(l) > 0.1)
    {
      count = 1;
      l = l + 0.0001;
    }
    else if(std::abs(l) < 0.15)
    {
      count = 0;
      l = l - 0.0001;
    }
    else if(std::abs(l) >= 0.15 && count == 0)
    {
      count = 0;
      l = l - 0.0001;
    }
    else if(std::abs(l) >= 0.15 && count == 1)
    {
      count = 1;
      l = l + 0.0001;
    }


    if (data == 0)
    {
      l = 0.15;
      data = 1;
    }
    else if (data == 1)
    {
      l = 0.1;
      data = 0;
    }
*/

//shack butt
/*
while(1)
  {
    if (getch() == ESC_ASCII_VALUE)
    {
      legged_bot.rest(portHandler, packetHandler, groupSyncWrite);
      break;

    }
      
    for(double i = 0; i <= 0.1; i = i+0.0001)
    {
      normal[0] = 0.1;
      normal[1] = i;

      point = body.plane(normal,l);
      legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
      usleep(sleep_time);
    }
    for(double i = 0; i <= 0.1; i = i+0.0001)
    {
      normal[0] = 0.1-i;
      normal[1] = 0.1;

      point = body.plane(normal,l);
      legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
      usleep(sleep_time);
    }
    for(double i = 0; i <= 0.1; i = i+0.0001)
    {
      normal[0] = -i;
      normal[1] = 0.1;

      point = body.plane(normal,l);
      legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
      usleep(sleep_time);
    }
    for(double i = 0; i <= 0.1; i = i+0.0001)
    {
      normal[0] = -0.1;
      normal[1] = 0.1-i;

      point = body.plane(normal,l);
      legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
      usleep(sleep_time);
    }
    for(double i = 0; i <= 0.1; i = i+0.0001)
    {
      normal[0] = -0.1;
      normal[1] = -i;

      point = body.plane(normal,l);
      legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
      usleep(sleep_time);
    }
    for(double i = 0; i <= 0.1; i = i+0.0001)
    {
      normal[0] = -0.1+i;
      normal[1] = -0.1;

      point = body.plane(normal,l);
      legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
      usleep(sleep_time);
    }
    for(double i = 0; i <= 0.1; i = i+0.0001)
    {
      normal[0] = i;
      normal[1] = -0.1;

      point = body.plane(normal,l);
      legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
      usleep(sleep_time);
    }
    for(double i = 0; i <= 0.1; i = i+0.0001)
    {
      normal[0] = 0.1;
      normal[1] = -0.1+i;

      point = body.plane(normal,l);
      legged_bot.moving(portHandler, packetHandler, groupSyncWrite, point);
      usleep(sleep_time);
    }

  }
  
  */