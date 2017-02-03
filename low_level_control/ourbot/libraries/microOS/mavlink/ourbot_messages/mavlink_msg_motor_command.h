// MESSAGE MOTOR_COMMAND PACKING

#define MAVLINK_MSG_ID_MOTOR_COMMAND 11

typedef struct __mavlink_motor_command_t
{
 int32_t command_left_front; ///< [Velocity] command for the left front wheel
 int32_t command_right_front; ///< [Velocity] command for the right front wheel
 int32_t command_left_rear; ///< [Velocity] command for the left rear wheel
 int32_t command_right_rear; ///< [Velocity] command for the right rear wheel
 uint8_t command_type; ///< Command type: can be used to change from velocity to current commands for example
} mavlink_motor_command_t;

#define MAVLINK_MSG_ID_MOTOR_COMMAND_LEN 17
#define MAVLINK_MSG_ID_11_LEN 17

#define MAVLINK_MSG_ID_MOTOR_COMMAND_CRC 55
#define MAVLINK_MSG_ID_11_CRC 55



#define MAVLINK_MESSAGE_INFO_MOTOR_COMMAND { \
	"MOTOR_COMMAND", \
	5, \
	{  { "command_left_front", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_motor_command_t, command_left_front) }, \
         { "command_right_front", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_motor_command_t, command_right_front) }, \
         { "command_left_rear", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_motor_command_t, command_left_rear) }, \
         { "command_right_rear", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_motor_command_t, command_right_rear) }, \
         { "command_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_motor_command_t, command_type) }, \
         } \
}


/**
 * @brief Pack a motor_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param command_type Command type: can be used to change from velocity to current commands for example
 * @param command_left_front [Velocity] command for the left front wheel
 * @param command_right_front [Velocity] command for the right front wheel
 * @param command_left_rear [Velocity] command for the left rear wheel
 * @param command_right_rear [Velocity] command for the right rear wheel
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t command_type, int32_t command_left_front, int32_t command_right_front, int32_t command_left_rear, int32_t command_right_rear)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOTOR_COMMAND_LEN];
	_mav_put_int32_t(buf, 0, command_left_front);
	_mav_put_int32_t(buf, 4, command_right_front);
	_mav_put_int32_t(buf, 8, command_left_rear);
	_mav_put_int32_t(buf, 12, command_right_rear);
	_mav_put_uint8_t(buf, 16, command_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#else
	mavlink_motor_command_t packet;
	packet.command_left_front = command_left_front;
	packet.command_right_front = command_right_front;
	packet.command_left_rear = command_left_rear;
	packet.command_right_rear = command_right_rear;
	packet.command_type = command_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOTOR_COMMAND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif
}

/**
 * @brief Pack a motor_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command_type Command type: can be used to change from velocity to current commands for example
 * @param command_left_front [Velocity] command for the left front wheel
 * @param command_right_front [Velocity] command for the right front wheel
 * @param command_left_rear [Velocity] command for the left rear wheel
 * @param command_right_rear [Velocity] command for the right rear wheel
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t command_type,int32_t command_left_front,int32_t command_right_front,int32_t command_left_rear,int32_t command_right_rear)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOTOR_COMMAND_LEN];
	_mav_put_int32_t(buf, 0, command_left_front);
	_mav_put_int32_t(buf, 4, command_right_front);
	_mav_put_int32_t(buf, 8, command_left_rear);
	_mav_put_int32_t(buf, 12, command_right_rear);
	_mav_put_uint8_t(buf, 16, command_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#else
	mavlink_motor_command_t packet;
	packet.command_left_front = command_left_front;
	packet.command_right_front = command_right_front;
	packet.command_left_rear = command_left_rear;
	packet.command_right_rear = command_right_rear;
	packet.command_type = command_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOTOR_COMMAND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif
}

/**
 * @brief Encode a motor_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param motor_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_motor_command_t* motor_command)
{
	return mavlink_msg_motor_command_pack(system_id, component_id, msg, motor_command->command_type, motor_command->command_left_front, motor_command->command_right_front, motor_command->command_left_rear, motor_command->command_right_rear);
}

/**
 * @brief Encode a motor_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_motor_command_t* motor_command)
{
	return mavlink_msg_motor_command_pack_chan(system_id, component_id, chan, msg, motor_command->command_type, motor_command->command_left_front, motor_command->command_right_front, motor_command->command_left_rear, motor_command->command_right_rear);
}

/**
 * @brief Send a motor_command message
 * @param chan MAVLink channel to send the message
 *
 * @param command_type Command type: can be used to change from velocity to current commands for example
 * @param command_left_front [Velocity] command for the left front wheel
 * @param command_right_front [Velocity] command for the right front wheel
 * @param command_left_rear [Velocity] command for the left rear wheel
 * @param command_right_rear [Velocity] command for the right rear wheel
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_motor_command_send(mavlink_channel_t chan, uint8_t command_type, int32_t command_left_front, int32_t command_right_front, int32_t command_left_rear, int32_t command_right_rear)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOTOR_COMMAND_LEN];
	_mav_put_int32_t(buf, 0, command_left_front);
	_mav_put_int32_t(buf, 4, command_right_front);
	_mav_put_int32_t(buf, 8, command_left_rear);
	_mav_put_int32_t(buf, 12, command_right_rear);
	_mav_put_uint8_t(buf, 16, command_type);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, buf, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, buf, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif
#else
	mavlink_motor_command_t packet;
	packet.command_left_front = command_left_front;
	packet.command_right_front = command_right_front;
	packet.command_left_rear = command_left_rear;
	packet.command_right_rear = command_right_rear;
	packet.command_type = command_type;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MOTOR_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_motor_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t command_type, int32_t command_left_front, int32_t command_right_front, int32_t command_left_rear, int32_t command_right_rear)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int32_t(buf, 0, command_left_front);
	_mav_put_int32_t(buf, 4, command_right_front);
	_mav_put_int32_t(buf, 8, command_left_rear);
	_mav_put_int32_t(buf, 12, command_right_rear);
	_mav_put_uint8_t(buf, 16, command_type);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, buf, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, buf, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif
#else
	mavlink_motor_command_t *packet = (mavlink_motor_command_t *)msgbuf;
	packet->command_left_front = command_left_front;
	packet->command_right_front = command_right_front;
	packet->command_left_rear = command_left_rear;
	packet->command_right_rear = command_right_rear;
	packet->command_type = command_type;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, (const char *)packet, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, (const char *)packet, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MOTOR_COMMAND UNPACKING


/**
 * @brief Get field command_type from motor_command message
 *
 * @return Command type: can be used to change from velocity to current commands for example
 */
static inline uint8_t mavlink_msg_motor_command_get_command_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field command_left_front from motor_command message
 *
 * @return [Velocity] command for the left front wheel
 */
static inline int32_t mavlink_msg_motor_command_get_command_left_front(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field command_right_front from motor_command message
 *
 * @return [Velocity] command for the right front wheel
 */
static inline int32_t mavlink_msg_motor_command_get_command_right_front(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field command_left_rear from motor_command message
 *
 * @return [Velocity] command for the left rear wheel
 */
static inline int32_t mavlink_msg_motor_command_get_command_left_rear(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field command_right_rear from motor_command message
 *
 * @return [Velocity] command for the right rear wheel
 */
static inline int32_t mavlink_msg_motor_command_get_command_right_rear(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Decode a motor_command message into a struct
 *
 * @param msg The message to decode
 * @param motor_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_motor_command_decode(const mavlink_message_t* msg, mavlink_motor_command_t* motor_command)
{
#if MAVLINK_NEED_BYTE_SWAP
	motor_command->command_left_front = mavlink_msg_motor_command_get_command_left_front(msg);
	motor_command->command_right_front = mavlink_msg_motor_command_get_command_right_front(msg);
	motor_command->command_left_rear = mavlink_msg_motor_command_get_command_left_rear(msg);
	motor_command->command_right_rear = mavlink_msg_motor_command_get_command_right_rear(msg);
	motor_command->command_type = mavlink_msg_motor_command_get_command_type(msg);
#else
	memcpy(motor_command, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif
}
