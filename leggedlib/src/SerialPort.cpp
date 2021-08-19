#include "SerialClass.h"
using namespace std;

Serial::Serial(int *serial_port, char port[], int baudrate)
{
    *serial_port = open(port, O_RDWR);
    termios tty = {};

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    if(baudrate == 9600)
    {
        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);
    }
    else if(baudrate == 115200)
    {
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
    }
    else if(baudrate == 57600)
    {
        cfsetispeed(&tty, B57600);
        cfsetospeed(&tty, B57600);
    }
    else if(baudrate == 1000000)
    {
        cfsetispeed(&tty, B1000000);
        cfsetospeed(&tty, B1000000);
    }
}

int Serial::readserial(int serial_port)
{
    string str;
    char chr;
    int num_bytes;
    int message;
    int state = 0;
    while(1)
    {
      num_bytes = read(serial_port, &chr, 1);
      if(chr == '\n')
        break;
      else
        str.push_back(chr);
    }
    if(sscanf(str.c_str(), "%d", &message) != 1);
    str = "";
    return message;
}

/*
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
    // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
    serial_port = open("/dev/ttyACM0", O_RDWR);

    if(flock(serial_port, LOCK_EX | LOCK_NB) == -1) {
        throw std::runtime_error("Serial port with file descriptor " + 
                                 std::to_string(serial_port) + " is already locked by another process.");
    }


    // Create new termios struct, we call it 'tty' for convention
    termios tty = {};

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    // Write to serial port
    const char* msg = "Hello from raspberry pi!\n";
    write(serial_port, msg, sizeof(msg));

    while (true) {
        // Read bytes. The behaviour of read() (e.g. does it block?,
        // how long does it block for?) depends on the configuration
        // settings above, specifically VMIN and VTIME
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

        // wait for 10ms before reading again,
        //   important to yield regularly so you don't lock up the CPU 
        nanosleep((const struct timespec[]){{0, 10000000L}}, NULL);
        // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
    }

    return 0;
}
*/