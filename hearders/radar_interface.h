/*
 * radar_interface.h
 *
 *  Created on: 12 Apr 2017
 *      Author: pi
 */

#ifndef RADAR_INTERFACE_H_
#define RADAR_INTERFACE_H_

#include "ArbeRoboticsRadar.h"
//#include "global.h"

int uart_init();

void get_uart_data();

//void Radar_Uart_handler(uint8_t* buff, int len);
void RadarError(RadarException exception);

void RadarStatusUpdated(RadarStatus status);

void TargetsMessageReceived(Target_Data *targets, int num_targets, int sector_id);

bool TransmitUartBuffer(uint8_t *buffer, int len);


#endif /* RADAR_INTERFACE_H_ */
