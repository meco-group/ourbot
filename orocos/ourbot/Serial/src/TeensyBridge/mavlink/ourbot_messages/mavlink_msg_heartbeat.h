// MESSAGE HEARTBEAT PACKING

#define MAVLINK_MSG_ID_HEARTBEAT 0

typedef struct __mavlink_heartbeat_t
{
 uint32_t mode; ///< A bitfield for use for autopilot-specific flags.
 uint32_t time; ///< System onboard time
 uint32_t event; ///< System status flag: an event has occurred
 int16_t VBat; ///< Battery voltage
 uint8_t type; ///< Type of ourbot
 uint8_t mavlink_version; ///< MAVLink version
} mavlink_heartbeat_t;

#define MAVLINK_MSG_ID_HEARTBEAT_LEN 16
#define MAVLINK_MSG_ID_0_LEN 16

#define MAVLINK_MSG_ID_HEARTBEAT_CRC 137
#define MAVLINK_MSG_ID_0_CRC 137



#define MAVLINK_MESSAGE_INFO_HEARTBEAT { \
	"HEARTBEAT", \
	6, \
	{  { "mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_heartbeat_t, mode) }, \
         { "time", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_heartbeat_t, time) }, \
         { "event", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_heartbeat_t, event) }, \
         { "VBat", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_heartbeat_t, VBat) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_heartbeat_t, type) }, \
         { "mavlink_version", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_heartbeat_t, mavlink_version) }, \
         } \
}


/**
 * @brief Pack a heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode A bitfield for use for autopilot-specific flags.
 * @param time System onboard time
 * @param VBat Battery voltage
 * @param type Type of ourbot
 * @param event System status flag: an event has occurred
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t mode, uint32_t time, int16_t VBat, uint8_t type, uint32_t event)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_HEARTBEAT_LEN];
	_mav_put_uint32_t(buf, 0, mode);
	_mav_put_uint32_t(buf, 4, time);
	_mav_put_uint32_t(buf, 8, event);
	_mav_put_int16_t(buf, 12, VBat);
	_mav_put_uint8_t(buf, 14, type);
	_mav_put_uint8_t(buf, 15, 2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#else
	mavlink_heartbeat_t packet;
	packet.mode = mode;
	packet.time = time;
	packet.event = event;
	packet.VBat = VBat;
	packet.type = type;
	packet.mavlink_version = 2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif
}

/**
 * @brief Pack a heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mode A bitfield for use for autopilot-specific flags.
 * @param time System onboard time
 * @param VBat Battery voltage
 * @param type Type of ourbot
 * @param event System status flag: an event has occurred
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t mode,uint32_t time,int16_t VBat,uint8_t type,uint32_t event)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_HEARTBEAT_LEN];
	_mav_put_uint32_t(buf, 0, mode);
	_mav_put_uint32_t(buf, 4, time);
	_mav_put_uint32_t(buf, 8, event);
	_mav_put_int16_t(buf, 12, VBat);
	_mav_put_uint8_t(buf, 14, type);
	_mav_put_uint8_t(buf, 15, 2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#else
	mavlink_heartbeat_t packet;
	packet.mode = mode;
	packet.time = time;
	packet.event = event;
	packet.VBat = VBat;
	packet.type = type;
	packet.mavlink_version = 2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif
}

/**
 * @brief Encode a heartbeat struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_heartbeat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_heartbeat_t* heartbeat)
{
	return mavlink_msg_heartbeat_pack(system_id, component_id, msg, heartbeat->mode, heartbeat->time, heartbeat->VBat, heartbeat->type, heartbeat->event);
}

/**
 * @brief Encode a heartbeat struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_heartbeat_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_heartbeat_t* heartbeat)
{
	return mavlink_msg_heartbeat_pack_chan(system_id, component_id, chan, msg, heartbeat->mode, heartbeat->time, heartbeat->VBat, heartbeat->type, heartbeat->event);
}

/**
 * @brief Send a heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param mode A bitfield for use for autopilot-specific flags.
 * @param time System onboard time
 * @param VBat Battery voltage
 * @param type Type of ourbot
 * @param event System status flag: an event has occurred
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_heartbeat_send(mavlink_channel_t chan, uint32_t mode, uint32_t time, int16_t VBat, uint8_t type, uint32_t event)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_HEARTBEAT_LEN];
	_mav_put_uint32_t(buf, 0, mode);
	_mav_put_uint32_t(buf, 4, time);
	_mav_put_uint32_t(buf, 8, event);
	_mav_put_int16_t(buf, 12, VBat);
	_mav_put_uint8_t(buf, 14, type);
	_mav_put_uint8_t(buf, 15, 2);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, buf, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, buf, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif
#else
	mavlink_heartbeat_t packet;
	packet.mode = mode;
	packet.time = time;
	packet.event = event;
	packet.VBat = VBat;
	packet.type = type;
	packet.mavlink_version = 2;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)&packet, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)&packet, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_HEARTBEAT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_heartbeat_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t mode, uint32_t time, int16_t VBat, uint8_t type, uint32_t event)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, mode);
	_mav_put_uint32_t(buf, 4, time);
	_mav_put_uint32_t(buf, 8, event);
	_mav_put_int16_t(buf, 12, VBat);
	_mav_put_uint8_t(buf, 14, type);
	_mav_put_uint8_t(buf, 15, 2);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, buf, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, buf, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif
#else
	mavlink_heartbeat_t *packet = (mavlink_heartbeat_t *)msgbuf;
	packet->mode = mode;
	packet->time = time;
	packet->event = event;
	packet->VBat = VBat;
	packet->type = type;
	packet->mavlink_version = 2;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)packet, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)packet, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE HEARTBEAT UNPACKING


/**
 * @brief Get field mode from heartbeat message
 *
 * @return A bitfield for use for autopilot-specific flags.
 */
static inline uint32_t mavlink_msg_heartbeat_get_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field time from heartbeat message
 *
 * @return System onboard time
 */
static inline uint32_t mavlink_msg_heartbeat_get_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field VBat from heartbeat message
 *
 * @return Battery voltage
 */
static inline int16_t mavlink_msg_heartbeat_get_VBat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field type from heartbeat message
 *
 * @return Type of ourbot
 */
static inline uint8_t mavlink_msg_heartbeat_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field event from heartbeat message
 *
 * @return System status flag: an event has occurred
 */
static inline uint32_t mavlink_msg_heartbeat_get_event(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field mavlink_version from heartbeat message
 *
 * @return MAVLink version
 */
static inline uint8_t mavlink_msg_heartbeat_get_mavlink_version(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Decode a heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param heartbeat C-struct to decode the message contents into
 */
static inline void mavlink_msg_heartbeat_decode(const mavlink_message_t* msg, mavlink_heartbeat_t* heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP
	heartbeat->mode = mavlink_msg_heartbeat_get_mode(msg);
	heartbeat->time = mavlink_msg_heartbeat_get_time(msg);
	heartbeat->event = mavlink_msg_heartbeat_get_event(msg);
	heartbeat->VBat = mavlink_msg_heartbeat_get_VBat(msg);
	heartbeat->type = mavlink_msg_heartbeat_get_type(msg);
	heartbeat->mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(msg);
#else
	memcpy(heartbeat, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif
}
