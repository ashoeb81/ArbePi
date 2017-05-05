/*
 * Arbe radar serial message structures.
 */

#ifndef SERIALINTERFACE_H_
#define SERIALINTERFACE_H_

/*
 *
 * Defines
 *
 */

#define UART_CMD_MSG_BUFFER_SIZE    2
#define MAX_PROCESSED_POINTS        64
#define RAW_MSG_DATA_POINTS            256
#define SAMPLES_PER_FRAME            512
#define MAX_MSG_LEN                    4096
#define UART_HEADER 0xA55A

/*
 *
 * Structs
 *
 */
struct __attribute__((__packed__)) EMPTY_msg {
    uint16_t header;
    uint8_t type;
    uint16_t len;
    uint8_t crc;
};

struct IF_data_msg {
    uint16_t sample_num;
    int16_t IF1_I;
    int16_t IF1_Q;
    int16_t IF2_I;
    int16_t IF2_Q;
};

struct __attribute__((__packed__)) RAW_msg {
    uint16_t header;
    uint8_t type;
    uint16_t len;
    uint8_t sector_id;
    uint8_t frame_num;
    uint16_t samples_per_buf;
    uint8_t buffer_number;
    float f_s;
    float T_chirp;
    uint8_t sweep_dir;
    IF_data_msg IFdata[RAW_MSG_DATA_POINTS];
    uint8_t crc;
};

struct DSP_Target_Data {
    float range;
    float dir;
    float vel;
    float prob;
    float amp;
    float snr;
};
struct Target_Data {
    float range;
    float dir;
    float vel;
    float prob;
    float amp;
    float snr;
};

struct __attribute__((__packed__)) Targets_msg {
    uint16_t header;
    uint8_t type;
    uint16_t len;
    uint32_t time;
    uint8_t sector_id;
    uint8_t num_targets;
    uint8_t packet_num;
    DSP_Target_Data targets_data[MAX_PROCESSED_POINTS];
    uint8_t crc;
};


struct __attribute__((__packed__))  Spectrum_msg {
    uint16_t header;
    uint8_t type;
    uint16_t len;
    uint8_t sector_id;
    uint16_t sweep_num;
    float sweep_beta;
    float sampling_rate;
    float amp[SAMPLES_PER_FRAME];
    float dir[SAMPLES_PER_FRAME / 2];
    uint8_t crc;
};

struct __attribute__((__packed__)) Status_msg {
    uint16_t header;
    uint8_t type;
    uint16_t len;
    uint32_t time;
    uint8_t active_sector_id;
    uint8_t calibrated_sector_id;
    uint32_t status_bit_mask;
    float temprature;
    float tx_power;
    uint8_t crc;
};


struct __attribute__((__packed__)) Uart_Cmd {
    uint16_t header;
    uint8_t cmd;
    uint8_t mode;
    uint8_t id;
    uint16_t bit_mask;
    uint16_t param1;
    uint8_t param2;
    uint8_t param3;
    uint8_t crc;

};

struct __attribute__((__packed__)) Response_msg {
    uint16_t header;
    uint8_t type;
    uint16_t len;
    uint32_t time;
    Uart_Cmd cmd;
    uint8_t crc;
};


/*
 *
 * Enums
 *
 */
enum OUTPUT_TYPE {
    Raw_Output,
    Spectrum_Output,
    Targets_Output,
    Status_Output,
    Response_Output
};
enum CMD_TYPE {
    Start_Tx,
    Stop_Tx,
    Set_Params,
    GetStatus,
    Reset
};
enum SYS_PARAMS {
    Frame_Length,
    Block_Params,
};
enum MODE_TYPE {
    Const_Freq,
    Full_Calib,
    FMCW
};
enum STATUS_TYPE {
    Connection,
    System,
    Sector
};
enum DSP_BIT_MASK {
    Output_bit0 = 0x0001,
    Output_bit1 = 0x0002,
    TxPowRed0 = 0x0004,
    TxPowRed1 = 0x0008,
    TxPowRed2 = 0x0010,
    Use_HG = 0x0020,
    Calibrate = 0x0040,
    Continuous = 0x0080,
    Send_Status = 0x0100
};

enum STATUS_BIT_MASK {
    is_transmitting = 0x0001,
    is_HG = 0x0002,
    error1 = 0x0004,
    error2 = 0x0008,
    error3 = 0x0010,
    error4 = 0x0020,
    error5 = 0x0040,
    error6 = 0x0080,
    error7 = 0x0100
};


#endif /* SERIALINTERFACE_H_ */
