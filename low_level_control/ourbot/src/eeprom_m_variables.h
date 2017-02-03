#ifndef EEPROM_M_TABLE_H
#define EEPROM_M_TABLE_H

#define EEPROM_INDEX	2
#define NUMBER_ENTRIES	10

#define EEPROM_TABLE \
    ENTRY(robot_id,				uint8_t) \
    ENTRY(IMU_acc_offs_x,		int16_t) \
    ENTRY(IMU_acc_offs_y,		int16_t) \
    ENTRY(IMU_acc_offs_z,		int16_t) \
    ENTRY(IMU_acc_scale_x,		uint16_t) \
    ENTRY(IMU_acc_scale_y,		uint16_t) \
    ENTRY(IMU_acc_scale_z,		uint16_t) \
    ENTRY(IMU_gyro_offs_x,		int16_t) \
    ENTRY(IMU_gyro_offs_y,		int16_t) \
    ENTRY(IMU_gyro_offs_z,		int16_t)
    
#endif //EEPROM_M_TABLE_H
