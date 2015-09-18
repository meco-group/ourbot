#ifndef GAMEPAD_PROTOCOL_TYPEDEF_H
#define GAMEPAD_PROTOCOL_TYPEDEF_H

#define GAMEPAD_EVENT_BUTTON         0x01    /* button pressed/released */
#define GAMEPAD_EVENT_AXIS           0x02    /* joystick moved */
#define GAMEPAD_EVENT_INIT           0x80    /* initial state of device */

#define BUTTON_A      0x00
#define BUTTON_B      0x01
#define BUTTON_X      0x02
#define BUTTON_Y      0x03
#define BUTTON_LB     0x04
#define BUTTON_RB     0x05
#define BUTTON_BCK    0x06
#define BUTTON_STRT   0x07
#define BUTTON_LOG    0x08
#define BUTTON_AXISL  0x09
#define BUTTON_AXISR  0x0A

#define AXIS_LEFTX    0x00
#define AXIS_LEFTY    0x01
#define AXIS_LT       0x02
#define AXIS_RIGHTX   0x03
#define AXIS_RIGHTY   0x04
#define AXIS_RT       0x05
#define AXIS_LR       0x06
#define AXIS_UD       0x07

#define MAXVALUE      32767


typedef struct _gamepad_event_t {
  uint32_t time;      /* event timestamp in milliseconds */
  int16_t value;      /* value */
  uint8_t type;       /* event type */
  uint8_t number;     /* axis/button number */
} __attribute__((packed)) gamepad_event_t;

// typedef struct _rplidar_cmd_packet_t {
//     uint8_t syncByte; //must be RPLIDAR_CMD_SYNC_BYTE
//     uint8_t cmd_flag;
//     uint8_t size;
//     uint8_t data[0];
// } __attribute__((packed)) rplidar_cmd_packet_t;


// typedef struct _rplidar_ans_header_t {
//     uint8_t  syncByte1; // must be RPLIDAR_ANS_SYNC_BYTE1
//     uint8_t  syncByte2; // must be RPLIDAR_ANS_SYNC_BYTE2
//     uint32_t size_q30_subtype; // see _u32 size:30; _u32 subType:2;
//     uint8_t  type;
// } __attribute__((packed)) rplidar_ans_header_t;

#endif //GAMEPAD_PROTOCOL_TYPEDEF_H
