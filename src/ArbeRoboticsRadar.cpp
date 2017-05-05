/*
 * ArbeRoboticsRadarInterface.cpp
 *
 *  Created on: Oct 21, 2016
 *      Author: Noam Arkind
 */

#include "ArbeRoboticsRadar.h"
#include "radar_interface.h"

Uart_Cmd cmd_msg;


ArbeRoboticsRadar::ArbeRoboticsRadar() {
    this->status_updated = false;
    this->has_msg = false;
    this->rx_index = 0;
}

bool ArbeRoboticsRadar::UartRxHandler(uint8_t *buffer, uint32_t len) {
    memcpy(&(this->rx_data_buff[rx_index]), buffer, len);
    if (len == 1) {
        if (rx_index == 0 && buffer[0] == (UART_HEADER & 0xFF)) {
            this->has_msg = true;
            rx_index++;
            return true;
        }
    }
    int start_ind = 0;
    uint32_t i = start_ind;
    if (!this->has_msg || rx_index == 1) {
        this->has_msg = false;
        uint32_t inds = ((len == 1) ? 1 : (len - 1));
        for (i = 0; i < inds; i++) {
            if ((this->rx_data_buff[i] == ((UART_HEADER & 0xFF))) &&
                (this->rx_data_buff[i + 1] == (((UART_HEADER >> 8) & 0xFF)))) {
                this->has_msg = true;
                memcpy(&this->rx_data_buff[rx_index], buffer + i, len - i);
                this->rx_index += len - i;
                break;
            }
        }
    } else {
        if (this->rx_index + len <= MAX_MSG_LEN) {
            memcpy(this->rx_data_buff + this->rx_index, buffer, len);
            this->rx_index += len;
        } else {
            memcpy(this->rx_data_buff + this->rx_index, buffer, MAX_MSG_LEN - this->rx_index);
            this->rx_index = MAX_MSG_LEN;
        }
    }
    while (this->has_msg) {
        if (this->rx_index >= sizeof(EMPTY_msg)) {
            EMPTY_msg *msg = (EMPTY_msg *) this->rx_data_buff;
            if (msg->len > MAX_MSG_LEN) {
                RadarException ex;
                this->radarError(ex);
                this->rx_index = 0;
                this->has_msg = false;
                return false;
            } else if (this->rx_index >= sizeof(EMPTY_msg) + msg->len) {
                this->parse_message(this->rx_data_buff, msg->type,
                                    this->calcCrc(((uint8_t * )(msg)), (msg->len + sizeof(EMPTY_msg) - 1)));
                this->has_msg = false;
            } else
                break;
            for (i = sizeof(EMPTY_msg) + msg->len; i < this->rx_index - 1; i++)
                if ((this->rx_data_buff[i] == ((UART_HEADER & 0xFF))) &&
                    (this->rx_data_buff[i + 1] == (((UART_HEADER >> 8) & 0xFF)))) {
                    this->has_msg = true;
                    memmove(this->rx_data_buff, this->rx_data_buff + i, this->rx_index - i);
                    this->rx_index -= i;
                    break;
                }
        } else
            break;
    }
    if (!this->has_msg)
        this->rx_index = 0;

    return true;
}

void ArbeRoboticsRadar::parse_message(uint8_t *buffer, uint8_t type, uint8_t crc) {
    Target_Data *targets;
    switch (type) {
        case Raw_Output:
            RAW_msg *raw_msg;
            raw_msg = (RAW_msg *) buffer;
            if (raw_msg->crc != crc) {
                RadarException ex;
                this->radarError(ex);
                break;
            }
            break;
        case Spectrum_Output:
            Spectrum_msg *spectrum_msg;
            spectrum_msg = (Spectrum_msg *) buffer;
            if (spectrum_msg->crc != crc) {
                RadarException ex;
                this->radarError(ex);
                break;
            }
            break;
        case Targets_Output:
            Targets_msg *targets_msg;
            targets_msg = (Targets_msg *) buffer;
            if (targets_msg->crc != crc) {
                RadarException ex;
                this->radarError(ex);
                break;
            }

            targets = new Target_Data[targets_msg->num_targets];
            for (int i = 0; i < targets_msg->num_targets; i++) {
                targets[i].range = targets_msg->targets_data[i].range;
                targets[i].dir = targets_msg->targets_data[i].dir;
                targets[i].vel = targets_msg->targets_data[i].vel;
                targets[i].prob = targets_msg->targets_data[i].prob;
            }
            this->targetsMessageRecieved(targets, targets_msg->num_targets, targets_msg->sector_id);
            delete (targets);
            break;
        case Status_Output:
            Status_msg *status_msg;
            status_msg = (Status_msg *) buffer;
            if (status_msg->crc != crc) {
                RadarException ex;
                ex.type = INVALID_MSG;
                strcpy(ex.msg, "CRC value incorrect for status message");
                this->radarError(ex);
                break;
            }
            this->currStatus.bitmask = status_msg->status_bit_mask;
            this->status_updated = true;
            this->statusUpdatedHandler(this->currStatus);
            break;
        case Response_Output: {
            Response_msg *resp_msg;
            resp_msg = (Response_msg *) buffer;
            if (resp_msg->crc != crc) {
                RadarException ex;
                ex.type = INVALID_MSG;
                strcpy(ex.msg, "CRC value incorrect for status message");
                this->radarError(ex);
                break;
            }
            this->response_updated = (memcmp(&resp_msg->cmd, &cmd_msg, sizeof(Uart_Cmd)) == 0);
            break;
        }
        default:
            RadarException ex;
            this->radarError(ex);
            break;
    }
}

bool ArbeRoboticsRadar::ConfigureRadar(RadarConfiguration config) {
    int timeout_counter = 100000, num_attempts = 50;
    if (!this->transmitUartBuffer)
        return false;
    this->response_updated = false;
    while (--num_attempts && !this->response_updated) {
        int timer = timeout_counter;
//		send_ack();
        if (!send_command(Set_Params, Frame_Length, config.sector_id, 0, config.num_of_blocks, 0,
                          0))//send num of blocks.
            return false;
        while (!this->response_updated && --timer) {}
        get_uart_data();
        if (!this->response_updated)
            send_ack();
    }

    for (uint32_t i = 0; i < config.num_of_blocks; i++) {    //send block params
        if (this->response_updated) {
            timeout_counter = 100000;
            num_attempts = 50;
            this->response_updated = false;
            while (--num_attempts && !this->response_updated) {
                int timer = timeout_counter;
                //			send_ack();
                if (!send_command(Set_Params, Block_Params, i, 0, config.block_params[i].T_sweep,
                                  config.block_params[i].non_co_avg_sweeps,
                                  config.block_params[i].co_avg_points))//send num of blocks.
                    //				if(!(--num_attempts))
                    return false;
                while (!this->response_updated && --timer) {}
                get_uart_data();

                if (!this->response_updated)
                    send_ack();
            }
        }
    }

    return this->response_updated;

}

bool ArbeRoboticsRadar::Connect(int timeout_counter, int num_attempts) {
    if (!this->transmitUartBuffer)
        return false;
    this->status_updated = false;
    while (--num_attempts && !this->status_updated) {
        int timer = timeout_counter;
        if (!send_command(GetStatus, Connection, 0, 0, 0, 0, 0))
            return false;
        while (!this->status_updated && --timer) {}
        get_uart_data();
        if (!this->status_updated)
            send_ack();
    }
    return this->status_updated;
}

RadarStatus *ArbeRoboticsRadar::getStatus(STATUS_TYPE type, int timeout_counter) {
    if (!this->transmitUartBuffer)
        return NULL;
    this->status_updated = false;
    if (!send_command(GetStatus, type, 0, 0, 0, 0, 0))
        return NULL;
    while (!this->status_updated && --timeout_counter) {}
    get_uart_data();
    if (!this->status_updated)
        return NULL;
    return &this->currStatus;
}

bool ArbeRoboticsRadar::StartRadar(RadarConfiguration config) {
    uint16_t bitmask = (config.output_type | Use_HG | (config.continues * Continuous) |
                        (config.with_calib * Calibrate));
    int timeout_counter = 100000, num_attempts = 50;
    if (!this->transmitUartBuffer)
        return false;
    this->response_updated = 0;
    while (--num_attempts && !this->response_updated) {
        int timer = timeout_counter;
        if (!send_command(Start_Tx, FMCW, config.sector_id, bitmask, 3, config.delayPoints, 10))//send start tx.
            return false;
        while (!this->response_updated && --timer) {}
        get_uart_data();

    }
    return this->status_updated;
}

bool ArbeRoboticsRadar::StopRadar() {
    int timeout_counter = 100000, num_attempts = 50;
    if (!this->transmitUartBuffer)
        return false;
    this->response_updated = false;
    while (--num_attempts && !this->response_updated) {
        int timer = timeout_counter;
        //		send_ack();
        if (!send_command(Stop_Tx, 0, 0, 0, 0, 0, 0))//send num of blocks.
            return false;
        while (!this->response_updated && --timer) {}
        get_uart_data();
        if (!this->response_updated)
            send_ack();
    }
    return this->response_updated;
}

bool ArbeRoboticsRadar::ChangeSector(int sectorId) {
    return true;
}

void ArbeRoboticsRadar::SignUartCmd(Uart_Cmd *cmd) {
    cmd->header = UART_HEADER;
    cmd->crc = this->calcCrc((uint8_t * ) & (cmd->header), sizeof(Uart_Cmd) - sizeof(cmd->crc));
}

uint8_t ArbeRoboticsRadar::calcCrc(uint8_t *buff, uint32_t len) {
    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++)
        crc ^= *(buff + i);
    return crc;
}

void ArbeRoboticsRadar::send_ack() {
    if (this->transmitUartBuffer)
        this->transmitUartBuffer((uint8_t *) "111111111111", 12);
}

bool ArbeRoboticsRadar::send_command(CMD_TYPE cmd, uint32_t mode, uint32_t id, uint16_t bit_mask, uint16_t param1, uint8_t param2,
                                     uint8_t param3) {
    cmd_msg.cmd = cmd;
    cmd_msg.mode = mode;
    cmd_msg.id = id;
    cmd_msg.bit_mask = bit_mask;
    cmd_msg.param1 = param1;
    cmd_msg.param2 = param2;
    cmd_msg.param3 = param3;
    SignUartCmd(&cmd_msg);
    return this->transmitUartBuffer((uint8_t * ) & cmd_msg, sizeof(Uart_Cmd));
}

