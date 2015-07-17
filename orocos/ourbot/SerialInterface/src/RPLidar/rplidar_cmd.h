/*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  RoboPeak LIDAR System
 *  Data Packet IO packet definition for RP-LIDAR
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  
 */


#ifndef RPLIDAR_CMD_H
#define RPLIDAR_CMD_H

#include "rplidar_protocol.h"

// Commands
//-----------------------------------------

// Commands without payload and response
#define RPLIDAR_CMD_STOP               0x25
#define RPLIDAR_CMD_SCAN               0x20
#define RPLIDAR_CMD_FORCE_SCAN         0x21
#define RPLIDAR_CMD_RESET              0x40


// Commands without payload but have response
#define RPLIDAR_CMD_GET_DEVICE_INFO      0x50
#define RPLIDAR_CMD_GET_DEVICE_HEALTH    0x52


// Response
// ------------------------------------------
#define RPLIDAR_ANS_TYPE_MEASUREMENT      0x81

#define RPLIDAR_ANS_TYPE_DEVINFO          0x4
#define RPLIDAR_ANS_TYPE_DEVHEALTH        0x6


#define RPLIDAR_STATUS_OK                 0x0
#define RPLIDAR_STATUS_WARNING            0x1
#define RPLIDAR_STATUS_ERROR              0x2

#define RPLIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define RPLIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1

typedef struct _rplidar_response_measurement_node_t {
    uint8_t    sync_quality;      // syncbit:1;syncbit_inverse:1;quality:6;
    uint16_t   angle_q6_checkbit; // check_bit:1;angle_q6:15;
    uint16_t   distance_q2;
} __attribute__((packed)) rplidar_response_measurement_node_t;

typedef struct _rplidar_response_device_info_t {
    uint8_t   model;
    uint16_t  firmware_version;
    uint8_t   hardware_version;
    uint8_t   serialnum[16];
} __attribute__((packed)) rplidar_response_device_info_t;

typedef struct _rplidar_response_device_health_t {
    uint8_t   status;
    uint16_t  error_code;
} __attribute__((packed)) rplidar_response_device_health_t;

#endif //RPLIDAR_CMD_H
