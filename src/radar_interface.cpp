/*
 * radar_interface.cpp
 *
 *  Created on: 12 Apr 2017
 *      Author: pi
 */

#include <wiringSerial.h>
#include <wiringPi.h>


#include <unistd.h>
#include "radar_interface.h"
#include <iostream>
#include <fstream>
#include <ctime>

using namespace std;

#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#define BUFFER_SIZE 1000

int fd;
ofstream logFile;

int uart_init() {

    if ((fd = serialOpen("/dev/ttyUSB0", 115200)) < 0) {
//	if((fd=serialOpen("/dev/ttyS0",115200))<0)
        std::cout << "Failed Serial Port opent!!!!" << strerror(errno) << endl;
        return -1;
    }
    if (wiringPiSetup() == (-1)) {
        std::cout << "Failed WiringPiSetup()" << std::endl;
        return -2;
    }
    std::cout << "Serial Port opent!!!!" << std::endl;
    logFile.open("/media/usb/log.txt", ios::out | ios::app);
    return 0;
}


bool TransmitUartBuffer(uint8_t *buffer, int len) {
    write(fd, buffer, len);
    serialFlush(fd);
    return true;
}

void RadarError(RadarException exception) {
    cout << "Radar Error!" << endl;
}

void RadarStatusUpdated(RadarStatus status) {
    cout << "Radar Status Updated!" << endl;
}

void TargetsMessageReceived(Target_Data *targets, int num_targets, int sector_id) {

    /*
    FLOAT range;
    FLOAT dir;
    FLOAT vel;
    FLOAT prob;
    FLOAT amp;
    FLOAT snr;
    */
    time_t now = time(0);
    char* dt = ctime(&now);
    cout << "Targets Message Received: " << num_targets << " " << sector_id << endl;
    for(int i=0; i < num_targets; i++) {
        logFile << "Target " << i 
                << "| Range " << targets[i].range 
                << "| Dir " << targets[i].dir
                << "| Velocity " << targets[i].vel
                << "| Timestamp " << dt
                << endl;
        cout << "Target " << i 
                << "| Range " << targets[i].range 
                << "| Dir " << targets[i].dir
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

