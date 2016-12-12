// MESSAGE RAW_IMU_DATA PACKING

#define MAVLINK_MSG_ID_RAW_IMU_DATA 20

typedef struct __mavlink_raw_imu_data_t
{
 int16_t acc[3]; ///< accelerometer data (x,y,z)
 int16_t gyro[3]; ///< gyroscope data (x,y,z)
 int16_t mag[3]; ///< magnetometer data (x,y,z)
} mavlink_raw_imu_data_t;

#define MAVLINK_MSG_ID_RAW_IMU_DATA_LEN 18
#define MAVLINK_MSG_ID_20_LEN 18

#define MAVLINK_MSG_ID_RAW_IMU_DATA_CRC 44
#define MAVLINK_MSG_ID_20_CRC 44

#define MAVLINK_MSG_RAW_IMU_DATA_FIELD_ACC_LEN 3
#define MAVLINK_MSG_RAW_IMU_DATA_FIELD_GYRO_LEN 3
#define MAVLINK_MSG_RAW_IMU_DATA_FIELD_MAG_LEN 3

#define MAVLINK_MESSAGE_INFO_RAW_IMU_DATA { \
	"RAW_IMU_DATA", \
	3, \
	{  { "acc", NULL, MAVLINK_TYPE_INT16_T, 3, 0, offsetof(mavlink_raw_imu_data_t, acc) }, \
         { "gyro", NULL, MAVLINK_TYPE_INT16_T, 3, 6, offsetof(mavlink_raw_imu_data_t, gyro) }, \
         { "mag", NULL, MAVLINK_TYPE_INT16_T, 3, 12, offsetof(mavlink_raw_imu_data_t, mag) }, \
         } \
}


/**
 * @brief Pack a raw_imu_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param acc accelerometer data (x,y,z)
 * @param gyro gyroscope data (x,y,z)
 * @param mag magnetometer data (x,y,z)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raw_imu_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const int16_t *acc, const int16_t *gyro, const int16_t *mag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RAW_IMU_DATA_LEN];

	_mav_put_int16_t_array(buf, 0, acc, 3);
	_mav_put_int16_t_array(buf, 6, gyro, 3);
	_mav_put_int16_t_array(buf, 12, mag, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RAW_IMU_DATA_LEN);
#else
	mavlink_raw_imu_data_t packet;

	mav_array_memcpy(packet.acc, acc, sizeof(int16_t)*3);
	mav_array_memcpy(packet.gyro, gyro, sizeof(int16_t)*3);
	mav_array_memcpy(packet.mag, mag, sizeof(int16_t)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RAW_IMU_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RAW_IMU_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RAW_IMU_DATA_LEN, MAVLINK_MSG_ID_RAW_IMU_DATA_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RAW_IMU_DATA_LEN);
#endif
}

/**
 * @brief Pack a raw_imu_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param acc accelerometer data (x,y,z)
 * @param gyro gyroscope data (x,y,z)
 * @param mag magnetometer data (x,y,z)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raw_imu_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const int16_t *acc,const int16_t *gyro,const int16_t *mag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RAW_IMU_DATA_LEN];

	_mav_put_int16_t_array(buf, 0, acc, 3);
	_mav_put_int16_t_array(buf, 6, gyro, 3);
	_mav_put_int16_t_array(buf, 12, mag, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RAW_IMU_DATA_LEN);
#else
	mavlink_raw_imu_data_t packet;

	mav_array_memcpy(packet.acc, acc, sizeof(int16_t)*3);
	mav_array_memcpy(packet.gyro, gyro, sizeof(int16_t)*3);
	mav_array_memcpy(packet.mag, mag, sizeof(int16_t)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RAW_IMU_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RAW_IMU_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RAW_IMU_DATA_LEN, MAVLINK_MSG_ID_RAW_IMU_DATA_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RAW_IMU_DATA_LEN);
#endif
}

/**
 * @brief Encode a raw_imu_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param raw_imu_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_raw_imu_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_raw_imu_data_t* raw_imu_data)
{
	return mavlink_msg_raw_imu_data_pack(system_id, component_id, msg, raw_imu_data->acc, raw_imu_data->gyro, raw_imu_data->mag);
}

/**
 * @brief Encode a raw_imu_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param raw_imu_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_raw_imu_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_raw_imu_data_t* raw_imu_data)
{
	return mavlink_msg_raw_imu_data_pack_chan(system_id, component_id, chan, msg, raw_imu_data->acc, raw_imu_data->gyro, raw_imu_data->mag);
}

/**
 * @brief Send a raw_imu_data message
 * @param chan MAVLink channel to send the message
 *
 * @param acc accelerometer data (x,y,z)
 * @param gyro gyroscope data (x,y,z)
 * @param mag magnetometer data (x,y,z)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_raw_imu_data_send(mavlink_channel_t chan, const int16_t *acc, const int16_t *gyro, const int16_t *mag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RAW_IMU_DATA_LEN];

	_mav_put_int16_t_array(buf, 0, acc, 3);
	_mav_put_int16_t_array(buf, 6, gyro, 3);
	_mav_put_int16_t_array(buf, 12, mag, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_IMU_DATA, buf, MAVLINK_MSG_ID_RAW_IMU_DATA_LEN, MAVLINK_MSG_ID_RAW_IMU_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_IMU_DATA, buf, MAVLINK_MSG_ID_RAW_IMU_DATA_LEN);
#endif
#else
	mavlink_raw_imu_data_t packet;

	mav_array_memcpy(packet.acc, acc, sizeof(int16_t)*3);
	mav_array_memcpy(packet.gyro, gyro, sizeof(int16_t)*3);
	mav_array_memcpy(packet.mag, mag, sizeof(int16_t)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_IMU_DATA, (const char *)&packet, MAVLINK_MSG_ID_RAW_IMU_DATA_LEN, MAVLINK_MSG_ID_RAW_IMU_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_IMU_DATA, (const char *)&packet, MAVLINK_MSG_ID_RAW_IMU_DATA_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_RAW_IMU_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_raw_imu_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const int16_t *acc, const int16_t *gyro, const int16_t *mag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;

	_mav_put_int16_t_array(buf, 0, acc, 3);
	_mav_put_int16_t_array(buf, 6, gyro, 3);
	_mav_put_int16_t_array(buf, 12, mag, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_IMU_DATA, buf, MAVLINK_MSG_ID_RAW_IMU_DATA_LEN, MAVLINK_MSG_ID_RAW_IMU_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_IMU_DATA, buf, MAVLINK_MSG_ID_RAW_IMU_DATA_LEN);
#endif
#else
	mavlink_raw_imu_data_t *packet = (mavlink_raw_imu_data_t *)msgbuf;

	mav_array_memcpy(packet->acc, acc, sizeof(int16_t)*3);
	mav_array_memcpy(packet->gyro, gyro, sizeof(int16_t)*3);
	mav_array_memcpy(packet->mag, mag, sizeof(int16_t)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_IMU_DATA, (const char *)packet, MAVLINK_MSG_ID_RAW_IMU_DATA_LEN, MAVLINK_MSG_ID_RAW_IMU_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_IMU_DATA, (const char *)packet, MAVLINK_MSG_ID_RAW_IMU_DATA_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE RAW_IMU_DATA UNPACKING


/**
 * @brief Get field acc from raw_imu_data message
 *
 * @return accelerometer data (x,y,z)
 */
static inline uint16_t mavlink_msg_raw_imu_data_get_acc(const mavlink_message_t* msg, int16_t *acc)
{
	return _MAV_RETURN_int16_t_array(msg, acc, 3,  0);
}

/**
 * @brief Get field gyro from raw_imu_data message
 *
 * @return gyroscope data (x,y,z)
 */
static inline uint16_t mavlink_msg_raw_imu_data_get_gyro(const mavlink_message_t* msg, int16_t *gyro)
{
	return _MAV_RETURN_int16_t_array(msg, gyro, 3,  6);
}

/**
 * @brief Get field mag from raw_imu_data message
 *
 * @return magnetometer data (x,y,z)
 */
static inline uint16_t mavlink_msg_raw_imu_data_get_mag(const mavlink_message_t* msg, int16_t *mag)
{
	return _MAV_RETURN_int16_t_array(msg, mag, 3,  12);
}

/**
 * @brief Decode a raw_imu_data message into a struct
 *
 * @param msg The message to decode
 * @param raw_imu_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_raw_imu_data_decode(const mavlink_message_t* msg, mavlink_raw_imu_data_t* raw_imu_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_raw_imu_data_get_acc(msg, raw_imu_data->acc);
	mavlink_msg_raw_imu_data_get_gyro(msg, raw_imu_data->gyro);
	mavlink_msg_raw_imu_data_get_mag(msg, raw_imu_data->mag);
#else
	memcpy(raw_imu_data, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_RAW_IMU_DATA_LEN);
#endif
}
