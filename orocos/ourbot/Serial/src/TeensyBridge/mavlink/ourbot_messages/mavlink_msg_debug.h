// MESSAGE DEBUG PACKING

#define MAVLINK_MSG_ID_DEBUG 1

typedef struct __mavlink_debug_t
{
 uint32_t time; ///< Time at which the message was sent
 float float1; ///< float1 of the debug message
 float float2; ///< float2 of the debug message
 float float3; ///< float3 of the debug message
 int32_t int1; ///< int1 of the debug message
 int32_t int2; ///< int2 of the debug message
 int32_t int3; ///< int3 of the debug message
} mavlink_debug_t;

#define MAVLINK_MSG_ID_DEBUG_LEN 28
#define MAVLINK_MSG_ID_1_LEN 28

#define MAVLINK_MSG_ID_DEBUG_CRC 247
#define MAVLINK_MSG_ID_1_CRC 247



#define MAVLINK_MESSAGE_INFO_DEBUG { \
	"DEBUG", \
	7, \
	{  { "time", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_debug_t, time) }, \
         { "float1", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_debug_t, float1) }, \
         { "float2", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_debug_t, float2) }, \
         { "float3", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_debug_t, float3) }, \
         { "int1", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_debug_t, int1) }, \
         { "int2", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_debug_t, int2) }, \
         { "int3", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_debug_t, int3) }, \
         } \
}


/**
 * @brief Pack a debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time Time at which the message was sent
 * @param float1 float1 of the debug message
 * @param float2 float2 of the debug message
 * @param float3 float3 of the debug message
 * @param int1 int1 of the debug message
 * @param int2 int2 of the debug message
 * @param int3 int3 of the debug message
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_debug_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time, float float1, float float2, float float3, int32_t int1, int32_t int2, int32_t int3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DEBUG_LEN];
	_mav_put_uint32_t(buf, 0, time);
	_mav_put_float(buf, 4, float1);
	_mav_put_float(buf, 8, float2);
	_mav_put_float(buf, 12, float3);
	_mav_put_int32_t(buf, 16, int1);
	_mav_put_int32_t(buf, 20, int2);
	_mav_put_int32_t(buf, 24, int3);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEBUG_LEN);
#else
	mavlink_debug_t packet;
	packet.time = time;
	packet.float1 = float1;
	packet.float2 = float2;
	packet.float3 = float3;
	packet.int1 = int1;
	packet.int2 = int2;
	packet.int3 = int3;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEBUG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DEBUG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DEBUG_LEN, MAVLINK_MSG_ID_DEBUG_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DEBUG_LEN);
#endif
}

/**
 * @brief Pack a debug message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time Time at which the message was sent
 * @param float1 float1 of the debug message
 * @param float2 float2 of the debug message
 * @param float3 float3 of the debug message
 * @param int1 int1 of the debug message
 * @param int2 int2 of the debug message
 * @param int3 int3 of the debug message
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_debug_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time,float float1,float float2,float float3,int32_t int1,int32_t int2,int32_t int3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DEBUG_LEN];
	_mav_put_uint32_t(buf, 0, time);
	_mav_put_float(buf, 4, float1);
	_mav_put_float(buf, 8, float2);
	_mav_put_float(buf, 12, float3);
	_mav_put_int32_t(buf, 16, int1);
	_mav_put_int32_t(buf, 20, int2);
	_mav_put_int32_t(buf, 24, int3);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEBUG_LEN);
#else
	mavlink_debug_t packet;
	packet.time = time;
	packet.float1 = float1;
	packet.float2 = float2;
	packet.float3 = float3;
	packet.int1 = int1;
	packet.int2 = int2;
	packet.int3 = int3;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEBUG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DEBUG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DEBUG_LEN, MAVLINK_MSG_ID_DEBUG_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DEBUG_LEN);
#endif
}

/**
 * @brief Encode a debug struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_debug_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_debug_t* debug)
{
	return mavlink_msg_debug_pack(system_id, component_id, msg, debug->time, debug->float1, debug->float2, debug->float3, debug->int1, debug->int2, debug->int3);
}

/**
 * @brief Encode a debug struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_debug_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_debug_t* debug)
{
	return mavlink_msg_debug_pack_chan(system_id, component_id, chan, msg, debug->time, debug->float1, debug->float2, debug->float3, debug->int1, debug->int2, debug->int3);
}

/**
 * @brief Send a debug message
 * @param chan MAVLink channel to send the message
 *
 * @param time Time at which the message was sent
 * @param float1 float1 of the debug message
 * @param float2 float2 of the debug message
 * @param float3 float3 of the debug message
 * @param int1 int1 of the debug message
 * @param int2 int2 of the debug message
 * @param int3 int3 of the debug message
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_debug_send(mavlink_channel_t chan, uint32_t time, float float1, float float2, float float3, int32_t int1, int32_t int2, int32_t int3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DEBUG_LEN];
	_mav_put_uint32_t(buf, 0, time);
	_mav_put_float(buf, 4, float1);
	_mav_put_float(buf, 8, float2);
	_mav_put_float(buf, 12, float3);
	_mav_put_int32_t(buf, 16, int1);
	_mav_put_int32_t(buf, 20, int2);
	_mav_put_int32_t(buf, 24, int3);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG, buf, MAVLINK_MSG_ID_DEBUG_LEN, MAVLINK_MSG_ID_DEBUG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG, buf, MAVLINK_MSG_ID_DEBUG_LEN);
#endif
#else
	mavlink_debug_t packet;
	packet.time = time;
	packet.float1 = float1;
	packet.float2 = float2;
	packet.float3 = float3;
	packet.int1 = int1;
	packet.int2 = int2;
	packet.int3 = int3;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG, (const char *)&packet, MAVLINK_MSG_ID_DEBUG_LEN, MAVLINK_MSG_ID_DEBUG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG, (const char *)&packet, MAVLINK_MSG_ID_DEBUG_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_DEBUG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_debug_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time, float float1, float float2, float float3, int32_t int1, int32_t int2, int32_t int3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time);
	_mav_put_float(buf, 4, float1);
	_mav_put_float(buf, 8, float2);
	_mav_put_float(buf, 12, float3);
	_mav_put_int32_t(buf, 16, int1);
	_mav_put_int32_t(buf, 20, int2);
	_mav_put_int32_t(buf, 24, int3);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG, buf, MAVLINK_MSG_ID_DEBUG_LEN, MAVLINK_MSG_ID_DEBUG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG, buf, MAVLINK_MSG_ID_DEBUG_LEN);
#endif
#else
	mavlink_debug_t *packet = (mavlink_debug_t *)msgbuf;
	packet->time = time;
	packet->float1 = float1;
	packet->float2 = float2;
	packet->float3 = float3;
	packet->int1 = int1;
	packet->int2 = int2;
	packet->int3 = int3;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG, (const char *)packet, MAVLINK_MSG_ID_DEBUG_LEN, MAVLINK_MSG_ID_DEBUG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG, (const char *)packet, MAVLINK_MSG_ID_DEBUG_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE DEBUG UNPACKING


/**
 * @brief Get field time from debug message
 *
 * @return Time at which the message was sent
 */
static inline uint32_t mavlink_msg_debug_get_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field float1 from debug message
 *
 * @return float1 of the debug message
 */
static inline float mavlink_msg_debug_get_float1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field float2 from debug message
 *
 * @return float2 of the debug message
 */
static inline float mavlink_msg_debug_get_float2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field float3 from debug message
 *
 * @return float3 of the debug message
 */
static inline float mavlink_msg_debug_get_float3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field int1 from debug message
 *
 * @return int1 of the debug message
 */
static inline int32_t mavlink_msg_debug_get_int1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field int2 from debug message
 *
 * @return int2 of the debug message
 */
static inline int32_t mavlink_msg_debug_get_int2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field int3 from debug message
 *
 * @return int3 of the debug message
 */
static inline int32_t mavlink_msg_debug_get_int3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Decode a debug message into a struct
 *
 * @param msg The message to decode
 * @param debug C-struct to decode the message contents into
 */
static inline void mavlink_msg_debug_decode(const mavlink_message_t* msg, mavlink_debug_t* debug)
{
#if MAVLINK_NEED_BYTE_SWAP
	debug->time = mavlink_msg_debug_get_time(msg);
	debug->float1 = mavlink_msg_debug_get_float1(msg);
	debug->float2 = mavlink_msg_debug_get_float2(msg);
	debug->float3 = mavlink_msg_debug_get_float3(msg);
	debug->int1 = mavlink_msg_debug_get_int1(msg);
	debug->int2 = mavlink_msg_debug_get_int2(msg);
	debug->int3 = mavlink_msg_debug_get_int3(msg);
#else
	memcpy(debug, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_DEBUG_LEN);
#endif
}
