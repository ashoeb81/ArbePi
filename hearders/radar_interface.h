/*
 * Arbe radar communications interface.
 */

#ifndef RADAR_INTERFACE_H_
#define RADAR_INTERFACE_H_

#include "ArbeRoboticsRadar.h"

/* Initialized serial port to communicate with Arbe radar.
 * @param string name of log file that saves Arbe radar messages.
 */
int uart_init(const string& log_fname);

/* Pulls data from serial port. */
void get_uart_data();

/* Callback when Arbe radar encounters an error.
 * @param RadarException message defined in ArbeRobotics.h
 */
void RadarError(RadarException exception);

/* Callback when Arbe radar status is updated.
 * @param RadarStatus message defined in ArbeRobotics.h
 */
void RadarStatusUpdated(RadarStatus status);

/* Callback when Arbe radar detects targets.
 * @param TargetData message defined in ArbeRobotics.h
 * @param integer number of targets detected.
 * @param integer designating antenna sector that made detections.
 */
void TargetsMessageReceived(Target_Data *targets, int num_targets, int sector_id);

/* Callback to send data to Arbe radar
 * @param byte array to transmit to Arbe radar.
 */
bool TransmitUartBuffer(uint8_t *buffer, int len);

#endif /* RADAR_INTERFACE_H_ */
