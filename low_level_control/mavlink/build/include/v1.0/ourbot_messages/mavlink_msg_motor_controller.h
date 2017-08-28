// MESSAGE MOTOR_CONTROLLER PACKING

#define MAVLINK_MSG_ID_MOTOR_CONTROLLER 12

typedef struct __mavlink_motor_controller_t
{
 float P; ///< Proportional controller gain
 float I; ///< Integral controller gain
 float D; ///< Derivative controller gain
} mavlink_motor_controller_t;

#define MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN 12
#define MAVLINK_MSG_ID_12_LEN 12

#define MAVLINK_MSG_ID_MOTOR_CONTROLLER_CRC 29
#define MAVLINK_MSG_ID_12_CRC 29



#define MAVLINK_MESSAGE_INFO_MOTOR_CONTROLLER { \
	"MOTOR_CONTROLLER", \
	3, \
	{  { "P", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_motor_controller_t, P) }, \
         { "I", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_motor_controller_t, I) }, \
         { "D", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_motor_controller_t, D) }, \
         } \
}


/**
 * @brief Pack a motor_controller message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param P Proportional controller gain
 * @param I Integral controller gain
 * @param D Derivative controller gain
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_controller_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float P, float I, float D)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN];
	_mav_put_float(buf, 0, P);
	_mav_put_float(buf, 4, I);
	_mav_put_float(buf, 8, D);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN);
#else
	mavlink_motor_controller_t packet;
	packet.P = P;
	packet.I = I;
	packet.D = D;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOTOR_CONTROLLER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN, MAVLINK_MSG_ID_MOTOR_CONTROLLER_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN);
#endif
}

/**
 * @brief Pack a motor_controller message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param P Proportional controller gain
 * @param I Integral controller gain
 * @param D Derivative controller gain
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_controller_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float P,float I,float D)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN];
	_mav_put_float(buf, 0, P);
	_mav_put_float(buf, 4, I);
	_mav_put_float(buf, 8, D);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN);
#else
	mavlink_motor_controller_t packet;
	packet.P = P;
	packet.I = I;
	packet.D = D;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOTOR_CONTROLLER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN, MAVLINK_MSG_ID_MOTOR_CONTROLLER_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN);
#endif
}

/**
 * @brief Encode a motor_controller struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param motor_controller C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_controller_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_motor_controller_t* motor_controller)
{
	return mavlink_msg_motor_controller_pack(system_id, component_id, msg, motor_controller->P, motor_controller->I, motor_controller->D);
}

/**
 * @brief Encode a motor_controller struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_controller C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_controller_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_motor_controller_t* motor_controller)
{
	return mavlink_msg_motor_controller_pack_chan(system_id, component_id, chan, msg, motor_controller->P, motor_controller->I, motor_controller->D);
}

/**
 * @brief Send a motor_controller message
 * @param chan MAVLink channel to send the message
 *
 * @param P Proportional controller gain
 * @param I Integral controller gain
 * @param D Derivative controller gain
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_motor_controller_send(mavlink_channel_t chan, float P, float I, float D)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN];
	_mav_put_float(buf, 0, P);
	_mav_put_float(buf, 4, I);
	_mav_put_float(buf, 8, D);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_CONTROLLER, buf, MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN, MAVLINK_MSG_ID_MOTOR_CONTROLLER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_CONTROLLER, buf, MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN);
#endif
#else
	mavlink_motor_controller_t packet;
	packet.P = P;
	packet.I = I;
	packet.D = D;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_CONTROLLER, (const char *)&packet, MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN, MAVLINK_MSG_ID_MOTOR_CONTROLLER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_CONTROLLER, (const char *)&packet, MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_motor_controller_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float P, float I, float D)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, P);
	_mav_put_float(buf, 4, I);
	_mav_put_float(buf, 8, D);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_CONTROLLER, buf, MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN, MAVLINK_MSG_ID_MOTOR_CONTROLLER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_CONTROLLER, buf, MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN);
#endif
#else
	mavlink_motor_controller_t *packet = (mavlink_motor_controller_t *)msgbuf;
	packet->P = P;
	packet->I = I;
	packet->D = D;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_CONTROLLER, (const char *)packet, MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN, MAVLINK_MSG_ID_MOTOR_CONTROLLER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_CONTROLLER, (const char *)packet, MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MOTOR_CONTROLLER UNPACKING


/**
 * @brief Get field P from motor_controller message
 *
 * @return Proportional controller gain
 */
static inline float mavlink_msg_motor_controller_get_P(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field I from motor_controller message
 *
 * @return Integral controller gain
 */
static inline float mavlink_msg_motor_controller_get_I(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field D from motor_controller message
 *
 * @return Derivative controller gain
 */
static inline float mavlink_msg_motor_controller_get_D(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a motor_controller message into a struct
 *
 * @param msg The message to decode
 * @param motor_controller C-struct to decode the message contents into
 */
static inline void mavlink_msg_motor_controller_decode(const mavlink_message_t* msg, mavlink_motor_controller_t* motor_controller)
{
#if MAVLINK_NEED_BYTE_SWAP
	motor_controller->P = mavlink_msg_motor_controller_get_P(msg);
	motor_controller->I = mavlink_msg_motor_controller_get_I(msg);
	motor_controller->D = mavlink_msg_motor_controller_get_D(msg);
#else
	memcpy(motor_controller, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MOTOR_CONTROLLER_LEN);
#endif
}
