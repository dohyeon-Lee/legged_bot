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

class Serial
{
    private:
        
    public:
        Serial(int *serial_port ,char port[], int baudrate);
        void writeserial(char arr[]);
        int readserial(int serial_port);
        int connect();
};
