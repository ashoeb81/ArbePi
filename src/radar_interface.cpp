#include <ctime>
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include "radar_interface.h"
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <wiringSerial.h>
#include <wiringPi.h>

// Maximum size of buffer to be received from Arbe radar.
#define BUFFER_SIZE 1000

using namespace std;

// file descriptor associated with serial port.
int fd;

// output file stream used to log Arbe radar targets.
ofstream logFile;

int uart_init(const string& log_fname) {
    if ((fd = serialOpen("/dev/ttyUSB0", 115200)) < 0) {
        std::cout << "Could not open serial port"
                  << strerror(errno)
                  << endl;
        return -1;
    }
    if (wiringPiSetup() == (-1)) {
        std::cout << "Failed to setup wiringPi"
                  << std::endl;
        return -2;
    }
    std::cout << "Serial port open"
              << std::endl;
    logFile.open(log_fname, ios::out);
    return 0;
}


bool TransmitUartBuffer(uint8_t *buffer, int len) {
    write(fd, buffer, len);
    serialFlush(fd);
    return true;
}

void RadarError(RadarException exception) {
    cout << "Radar Error Callback." << endl;
}

void RadarStatusUpdated(RadarStatus status) {
    cout << "Radar Status Callback." << endl;
}

void TargetsMessageReceived(Target_Data *targets, int num_targets, int sector_id) {
    // Get timestamp associated with target detections.
    time_t now = time(0);
    char *dt = ctime(&now);
    cout << num_targets << " Detected from sector " << sector_id << endl;
    // Log detection parameters associated with each target disk.
    for (int i = 0; i < num_targets; i++) {
        logFile << "Target " << i
                << "| Range " << targets[i].range
                << "| Direction " << targets[i].dir
                << "| Velocity " << targets[i].vel
                << "| Timestamp " << dt
                << endl;
    }

}

void get_uart_data() {
    uint8_t buffer[BUFFER_SIZE];
    int index = 0;
    int bytes_to_read = serialDataAvail(fd);
    while (bytes_to_read > 0) {
        if (index < BUFFER_SIZE) {
            buffer[index++] = serialGetchar(fd);
        } else {
            serialFlush(fd);
        }
        bytes_to_read--;

    }
    if (index > 0)
        AR_RADAR->UartRxHandler(buffer, index);
}

