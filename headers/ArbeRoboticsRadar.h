/*
 * Arbe radar API.
 */

#ifndef ARBEROBOTICSRADARintERFACE_H_
#define ARBEROBOTICSRADARintERFACE_H_

using namespace std;

#include "stdint.h"
#include "cstring"
#include "SerialInterface.h"
#include "RadarConfigurations.h"
//#include "radar_interface.h"

#define AR_RADAR ArbeRoboticsRadar::GetInstance()
#define MAX_EXCEPTION_MSG    100
#define MAX_TARGETS 100

struct RadarStatus {
    int type;
    uint16_t bitmask;
};

enum RadarExceptionType {
    INVALID_MSG
};

struct RadarException {
    RadarExceptionType type;
    char msg[MAX_EXCEPTION_MSG];
};

class ArbeRoboticsRadar {
private:
    ArbeRoboticsRadar();

    bool (*transmitUartBuffer)(uint8_t *buffer, int len);

    void (*targetsMessageRecieved)(Target_Data *targets, int num_targets, int sector_id);

    void (*statusUpdatedHandler)(RadarStatus status);

    void (*radarError)(RadarException exception);

    void SignUartCmd(Uart_Cmd *);

    bool status_updated;
    bool response_updated;
    uint8_t rx_data_buff[MAX_MSG_LEN];
    bool has_msg;
    uint8_t msg_type;
    uint16_t msg_len;
    uint32_t rx_index;

    uint8_t calcCrc(uint8_t *buff, uint32_t len);

    void send_ack();

    void parse_message(uint8_t *buffer, uint8_t type, uint8_t crc);

    bool
    send_command(CMD_TYPE cmd, uint32_t mode, uint32_t id, uint16_t bit_mask, uint16_t param1, uint8_t param2, uint8_t param3);

public:
    Target_Data targets[MAX_TARGETS];
    int num_targets;
    RadarStatus currStatus;
    RadarConfiguration currentConfiguration;


    bool UartRxHandler(uint8_t *buffer, uint32_t len);

    bool ConfigureRadar(RadarConfiguration config);

    bool Connect(int timeout_counter, int num_attempts);

    RadarStatus *getStatus(STATUS_TYPE type, int timeout_counter);

    bool StartRadar(RadarConfiguration config);

    bool StopRadar();

    bool ChangeSector(int sectorId);

    void SetTransmitBufferHandler(
            bool (*transmitUartBuffer_h)(uint8_t *buffer, int len)) { this->transmitUartBuffer = transmitUartBuffer_h; }

    void SetTargetsMessageReceivedHandler(void (*targetsMessageRecieved_h)(Target_Data *targets, int num_targets,
                                                                           int sector_id)) { this->targetsMessageRecieved = targetsMessageRecieved_h; }

    void SetStatusUpdatedHandler(
            void (*statusUpdated_h)(RadarStatus status)) { this->statusUpdatedHandler = statusUpdated_h; }

    void SetErrorHandler(void (*radarError_h)(RadarException exception)) { this->radarError = radarError_h; }

    static ArbeRoboticsRadar *GetInstance() {
        static ArbeRoboticsRadar *instance; // Guaranteed to be destroyed.
        // Instantiated on first use.
        if (instance == NULL)
            instance = new ArbeRoboticsRadar();
        return instance;
    }
};


#endif /* ARBEROBOTICSRADARintERFACE_H_ */

