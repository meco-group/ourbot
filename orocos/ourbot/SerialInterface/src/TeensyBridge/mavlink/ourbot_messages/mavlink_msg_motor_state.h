// MESSAGE MOTOR_STATE PACKING

#define MAVLINK_MSG_ID_MOTOR_STATE 10

typedef struct __mavlink_motor_state_t
{
 int32_t position; ///< Position of the motor in encoder ticks
 int32_t velocity; ///< Velocity of the motor in encoder ticks/sec
 int32_t acceleration; ///< Acceleration of the motor in encoder ticks/sec2
 int32_t current; ///< Armature current [mA]
 int32_t reference; ///< Reference for the controller. Convenience field when interested in closed loop response
 int16_t FFvoltage; ///< Feedforward armature voltage
 int16_t FBvoltage; ///< Feedback armature voltage
} mavlink_motor_state_t;

#define MAVLINK_MSG_ID_MOTOR_STATE_LEN 24
#define MAVLINK_MSG_ID_10_LEN 24

#define MAVLINK_MSG_ID_MOTOR_STATE_CRC 135
#define MAVLINK_MSG_ID_10_CRC 135



#define MAVLINK_MESSAGE_INFO_MOTOR_STATE { \
	"MOTOR_STATE", \
	7, \
	{  { "position", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_motor_state_t, position) }, \
         { "velocity", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_motor_state_t, velocity) }, \
         { "acceleration", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_motor_state_t, acceleration) }, \
         { "current", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_motor_state_t, current) }, \
         { "reference", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_motor_state_t, reference) }, \
         { "FFvoltage", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_motor_state_t, FFvoltage) }, \
         { "FBvoltage", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_motor_state_t, FBvoltage) }, \
         } \
}


/**
 * @brief Pack a motor_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param position Position of the motor in encoder ticks
 * @param velocity Velocity of the motor in encoder ticks/sec
 * @param acceleration Acceleration of the motor in encoder ticks/sec2
 * @param current Armature current [mA]
 * @param FFvoltage Feedforward armature voltage
 * @param FBvoltage Feedback armature voltage
 * @param reference Reference for the controller. Convenience field when interested in closed loop response
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int32_t position, int32_t velocity, int32_t acceleration, int32_t current, int16_t FFvoltage, int16_t FBvoltage, int32_t reference)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOTOR_STATE_LEN];
	_mav_put_int32_t(buf, 0, position);
	_mav_put_int32_t(buf, 4, velocity);
	_mav_put_int32_t(buf, 8, acceleration);
	_mav_put_int32_t(buf, 12, current);
	_mav_put_int32_t(buf, 16, reference);
	_mav_put_int16_t(buf, 20, FFvoltage);
	_mav_put_int16_t(buf, 22, FBvoltage);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_STATE_LEN);
#else
	mavlink_motor_state_t packet;
	packet.position = position;
	packet.velocity = velocity;
	packet.acceleration = acceleration;
	packet.current = current;
	packet.reference = reference;
	packet.FFvoltage = FFvoltage;
	packet.FBvoltage = FBvoltage;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_STATE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOTOR_STATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOTOR_STATE_LEN, MAVLINK_MSG_ID_MOTOR_STATE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOTOR_STATE_LEN);
#endif
}

/**
 * @brief Pack a motor_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param position Position of the motor in encoder ticks
 * @param velocity Velocity of the motor in encoder ticks/sec
 * @param acceleration Acceleration of the motor in encoder ticks/sec2
 * @param current Armature current [mA]
 * @param FFvoltage Feedforward armature voltage
 * @param FBvoltage Feedback armature voltage
 * @param reference Reference for the controller. Convenience field when interested in closed loop response
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int32_t position,int32_t velocity,int32_t acceleration,int32_t current,int16_t FFvoltage,int16_t FBvoltage,int32_t reference)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOTOR_STATE_LEN];
	_mav_put_int32_t(buf, 0, position);
	_mav_put_int32_t(buf, 4, velocity);
	_mav_put_int32_t(buf, 8, acceleration);
	_mav_put_int32_t(buf, 12, current);
	_mav_put_int32_t(buf, 16, reference);
	_mav_put_int16_t(buf, 20, FFvoltage);
	_mav_put_int16_t(buf, 22, FBvoltage);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_STATE_LEN);
#else
	mavlink_motor_state_t packet;
	packet.position = position;
	packet.velocity = velocity;
	packet.acceleration = acceleration;
	packet.current = current;
	packet.reference = reference;
	packet.FFvoltage = FFvoltage;
	packet.FBvoltage = FBvoltage;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_STATE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOTOR_STATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOTOR_STATE_LEN, MAVLINK_MSG_ID_MOTOR_STATE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOTOR_STATE_LEN);
#endif
}

/**
 * @brief Encode a motor_state struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param motor_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_motor_state_t* motor_state)
{
	return mavlink_msg_motor_state_pack(system_id, component_id, msg, motor_state->position, motor_state->velocity, motor_state->acceleration, motor_state->current, motor_state->FFvoltage, motor_state->FBvoltage, motor_state->reference);
}

/**
 * @brief Encode a motor_state struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_state_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_motor_state_t* motor_state)
{
	return mavlink_msg_motor_state_pack_chan(system_id, component_id, chan, msg, motor_state->position, motor_state->velocity, motor_state->acceleration, motor_state->current, motor_state->FFvoltage, motor_state->FBvoltage, motor_state->reference);
}

/**
 * @brief Send a motor_state message
 * @param chan MAVLink channel to send the message
 *
 * @param position Position of the motor in encoder ticks
 * @param velocity Velocity of the motor in encoder ticks/sec
 * @param acceleration Acceleration of the motor in encoder ticks/sec2
 * @param current Armature current [mA]
 * @param FFvoltage Feedforward armature voltage
 * @param FBvoltage Feedback armature voltage
 * @param reference Reference for the controller. Convenience field when interested in closed loop response
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_motor_state_send(mavlink_channel_t chan, int32_t position, int32_t velocity, int32_t acceleration, int32_t current, int16_t FFvoltage, int16_t FBvoltage, int32_t reference)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MOTOR_STATE_LEN];
	_mav_put_int32_t(buf, 0, position);
	_mav_put_int32_t(buf, 4, velocity);
	_mav_put_int32_t(buf, 8, acceleration);
	_mav_put_int32_t(buf, 12, current);
	_mav_put_int32_t(buf, 16, reference);
	_mav_put_int16_t(buf, 20, FFvoltage);
	_mav_put_int16_t(buf, 22, FBvoltage);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATE, buf, MAVLINK_MSG_ID_MOTOR_STATE_LEN, MAVLINK_MSG_ID_MOTOR_STATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATE, buf, MAVLINK_MSG_ID_MOTOR_STATE_LEN);
#endif
#else
	mavlink_motor_state_t packet;
	packet.position = position;
	packet.velocity = velocity;
	packet.acceleration = acceleration;
	packet.current = current;
	packet.reference = reference;
	packet.FFvoltage = FFvoltage;
	packet.FBvoltage = FBvoltage;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATE, (const char *)&packet, MAVLINK_MSG_ID_MOTOR_STATE_LEN, MAVLINK_MSG_ID_MOTOR_STATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATE, (const char *)&packet, MAVLINK_MSG_ID_MOTOR_STATE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MOTOR_STATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_motor_state_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t position, int32_t velocity, int32_t acceleration, int32_t current, int16_t FFvoltage, int16_t FBvoltage, int32_t reference)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int32_t(buf, 0, position);
	_mav_put_int32_t(buf, 4, velocity);
	_mav_put_int32_t(buf, 8, acceleration);
	_mav_put_int32_t(buf, 12, current);
	_mav_put_int32_t(buf, 16, reference);
	_mav_put_int16_t(buf, 20, FFvoltage);
	_mav_put_int16_t(buf, 22, FBvoltage);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATE, buf, MAVLINK_MSG_ID_MOTOR_STATE_LEN, MAVLINK_MSG_ID_MOTOR_STATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATE, buf, MAVLINK_MSG_ID_MOTOR_STATE_LEN);
#endif
#else
	mavlink_motor_state_t *packet = (mavlink_motor_state_t *)msgbuf;
	packet->position = position;
	packet->velocity = velocity;
	packet->acceleration = acceleration;
	packet->current = current;
	packet->reference = reference;
	packet->FFvoltage = FFvoltage;
	packet->FBvoltage = FBvoltage;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATE, (const char *)packet, MAVLINK_MSG_ID_MOTOR_STATE_LEN, MAVLINK_MSG_ID_MOTOR_STATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATE, (const char *)packet, MAVLINK_MSG_ID_MOTOR_STATE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MOTOR_STATE UNPACKING


/**
 * @brief Get field position from motor_state message
 *
 * @return Position of the motor in encoder ticks
 */
static inline int32_t mavlink_msg_motor_state_get_position(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field velocity from motor_state message
 *
 * @return Velocity of the motor in encoder ticks/sec
 */
static inline int32_t mavlink_msg_motor_state_get_velocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field acceleration from motor_state message
 *
 * @return Acceleration of the motor in encoder ticks/sec2
 */
static inline int32_t mavlink_msg_motor_state_get_acceleration(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field current from motor_state message
 *
 * @return Armature current [mA]
 */
static inline int32_t mavlink_msg_motor_state_get_current(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field FFvoltage from motor_state message
 *
 * @return Feedforward armature voltage
 */
static inline int16_t mavlink_msg_motor_state_get_FFvoltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field FBvoltage from motor_state message
 *
 * @return Feedback armature voltage
 */
static inline int16_t mavlink_msg_motor_state_get_FBvoltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field reference from motor_state message
 *
 * @return Reference for the controller. Convenience field when interested in closed loop response
 */
static inline int32_t mavlink_msg_motor_state_get_reference(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Decode a motor_state message into a struct
 *
 * @param msg The message to decode
 * @param motor_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_motor_state_decode(const mavlink_message_t* msg, mavlink_motor_state_t* motor_state)
{
#if MAVLINK_NEED_BYTE_SWAP
	motor_state->position = mavlink_msg_motor_state_get_position(msg);
	motor_state->velocity = mavlink_msg_motor_state_get_velocity(msg);
	motor_state->acceleration = mavlink_msg_motor_state_get_acceleration(msg);
	motor_state->current = mavlink_msg_motor_state_get_current(msg);
	motor_state->reference = mavlink_msg_motor_state_get_reference(msg);
	motor_state->FFvoltage = mavlink_msg_motor_state_get_FFvoltage(msg);
	motor_state->FBvoltage = mavlink_msg_motor_state_get_FBvoltage(msg);
#else
	memcpy(motor_state, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MOTOR_STATE_LEN);
#endif
}
