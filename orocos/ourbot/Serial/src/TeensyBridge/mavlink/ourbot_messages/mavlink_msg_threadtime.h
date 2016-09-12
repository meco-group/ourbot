// MESSAGE THREADTIME PACKING

#define MAVLINK_MSG_ID_THREADTIME 2

typedef struct __mavlink_threadtime_t
{
 uint32_t time; ///< Time at which the message was sent
 uint32_t thread1; ///< execution time of thread 1
 uint32_t thread2; ///< execution time of thread 2
 uint32_t thread3; ///< execution time of thread 3
 uint32_t thread4; ///< execution time of thread 4
 uint32_t thread5; ///< execution time of thread 5
 uint32_t thread6; ///< execution time of thread 6
} mavlink_threadtime_t;

#define MAVLINK_MSG_ID_THREADTIME_LEN 28
#define MAVLINK_MSG_ID_2_LEN 28

#define MAVLINK_MSG_ID_THREADTIME_CRC 40
#define MAVLINK_MSG_ID_2_CRC 40



#define MAVLINK_MESSAGE_INFO_THREADTIME { \
	"THREADTIME", \
	7, \
	{  { "time", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_threadtime_t, time) }, \
         { "thread1", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_threadtime_t, thread1) }, \
         { "thread2", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_threadtime_t, thread2) }, \
         { "thread3", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_threadtime_t, thread3) }, \
         { "thread4", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_threadtime_t, thread4) }, \
         { "thread5", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_threadtime_t, thread5) }, \
         { "thread6", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_threadtime_t, thread6) }, \
         } \
}


/**
 * @brief Pack a threadtime message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time Time at which the message was sent
 * @param thread1 execution time of thread 1
 * @param thread2 execution time of thread 2
 * @param thread3 execution time of thread 3
 * @param thread4 execution time of thread 4
 * @param thread5 execution time of thread 5
 * @param thread6 execution time of thread 6
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_threadtime_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time, uint32_t thread1, uint32_t thread2, uint32_t thread3, uint32_t thread4, uint32_t thread5, uint32_t thread6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_THREADTIME_LEN];
	_mav_put_uint32_t(buf, 0, time);
	_mav_put_uint32_t(buf, 4, thread1);
	_mav_put_uint32_t(buf, 8, thread2);
	_mav_put_uint32_t(buf, 12, thread3);
	_mav_put_uint32_t(buf, 16, thread4);
	_mav_put_uint32_t(buf, 20, thread5);
	_mav_put_uint32_t(buf, 24, thread6);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_THREADTIME_LEN);
#else
	mavlink_threadtime_t packet;
	packet.time = time;
	packet.thread1 = thread1;
	packet.thread2 = thread2;
	packet.thread3 = thread3;
	packet.thread4 = thread4;
	packet.thread5 = thread5;
	packet.thread6 = thread6;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_THREADTIME_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_THREADTIME;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_THREADTIME_LEN, MAVLINK_MSG_ID_THREADTIME_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_THREADTIME_LEN);
#endif
}

/**
 * @brief Pack a threadtime message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time Time at which the message was sent
 * @param thread1 execution time of thread 1
 * @param thread2 execution time of thread 2
 * @param thread3 execution time of thread 3
 * @param thread4 execution time of thread 4
 * @param thread5 execution time of thread 5
 * @param thread6 execution time of thread 6
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_threadtime_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time,uint32_t thread1,uint32_t thread2,uint32_t thread3,uint32_t thread4,uint32_t thread5,uint32_t thread6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_THREADTIME_LEN];
	_mav_put_uint32_t(buf, 0, time);
	_mav_put_uint32_t(buf, 4, thread1);
	_mav_put_uint32_t(buf, 8, thread2);
	_mav_put_uint32_t(buf, 12, thread3);
	_mav_put_uint32_t(buf, 16, thread4);
	_mav_put_uint32_t(buf, 20, thread5);
	_mav_put_uint32_t(buf, 24, thread6);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_THREADTIME_LEN);
#else
	mavlink_threadtime_t packet;
	packet.time = time;
	packet.thread1 = thread1;
	packet.thread2 = thread2;
	packet.thread3 = thread3;
	packet.thread4 = thread4;
	packet.thread5 = thread5;
	packet.thread6 = thread6;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_THREADTIME_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_THREADTIME;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_THREADTIME_LEN, MAVLINK_MSG_ID_THREADTIME_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_THREADTIME_LEN);
#endif
}

/**
 * @brief Encode a threadtime struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param threadtime C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_threadtime_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_threadtime_t* threadtime)
{
	return mavlink_msg_threadtime_pack(system_id, component_id, msg, threadtime->time, threadtime->thread1, threadtime->thread2, threadtime->thread3, threadtime->thread4, threadtime->thread5, threadtime->thread6);
}

/**
 * @brief Encode a threadtime struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param threadtime C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_threadtime_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_threadtime_t* threadtime)
{
	return mavlink_msg_threadtime_pack_chan(system_id, component_id, chan, msg, threadtime->time, threadtime->thread1, threadtime->thread2, threadtime->thread3, threadtime->thread4, threadtime->thread5, threadtime->thread6);
}

/**
 * @brief Send a threadtime message
 * @param chan MAVLink channel to send the message
 *
 * @param time Time at which the message was sent
 * @param thread1 execution time of thread 1
 * @param thread2 execution time of thread 2
 * @param thread3 execution time of thread 3
 * @param thread4 execution time of thread 4
 * @param thread5 execution time of thread 5
 * @param thread6 execution time of thread 6
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_threadtime_send(mavlink_channel_t chan, uint32_t time, uint32_t thread1, uint32_t thread2, uint32_t thread3, uint32_t thread4, uint32_t thread5, uint32_t thread6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_THREADTIME_LEN];
	_mav_put_uint32_t(buf, 0, time);
	_mav_put_uint32_t(buf, 4, thread1);
	_mav_put_uint32_t(buf, 8, thread2);
	_mav_put_uint32_t(buf, 12, thread3);
	_mav_put_uint32_t(buf, 16, thread4);
	_mav_put_uint32_t(buf, 20, thread5);
	_mav_put_uint32_t(buf, 24, thread6);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THREADTIME, buf, MAVLINK_MSG_ID_THREADTIME_LEN, MAVLINK_MSG_ID_THREADTIME_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THREADTIME, buf, MAVLINK_MSG_ID_THREADTIME_LEN);
#endif
#else
	mavlink_threadtime_t packet;
	packet.time = time;
	packet.thread1 = thread1;
	packet.thread2 = thread2;
	packet.thread3 = thread3;
	packet.thread4 = thread4;
	packet.thread5 = thread5;
	packet.thread6 = thread6;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THREADTIME, (const char *)&packet, MAVLINK_MSG_ID_THREADTIME_LEN, MAVLINK_MSG_ID_THREADTIME_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THREADTIME, (const char *)&packet, MAVLINK_MSG_ID_THREADTIME_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_THREADTIME_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_threadtime_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time, uint32_t thread1, uint32_t thread2, uint32_t thread3, uint32_t thread4, uint32_t thread5, uint32_t thread6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time);
	_mav_put_uint32_t(buf, 4, thread1);
	_mav_put_uint32_t(buf, 8, thread2);
	_mav_put_uint32_t(buf, 12, thread3);
	_mav_put_uint32_t(buf, 16, thread4);
	_mav_put_uint32_t(buf, 20, thread5);
	_mav_put_uint32_t(buf, 24, thread6);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THREADTIME, buf, MAVLINK_MSG_ID_THREADTIME_LEN, MAVLINK_MSG_ID_THREADTIME_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THREADTIME, buf, MAVLINK_MSG_ID_THREADTIME_LEN);
#endif
#else
	mavlink_threadtime_t *packet = (mavlink_threadtime_t *)msgbuf;
	packet->time = time;
	packet->thread1 = thread1;
	packet->thread2 = thread2;
	packet->thread3 = thread3;
	packet->thread4 = thread4;
	packet->thread5 = thread5;
	packet->thread6 = thread6;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THREADTIME, (const char *)packet, MAVLINK_MSG_ID_THREADTIME_LEN, MAVLINK_MSG_ID_THREADTIME_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THREADTIME, (const char *)packet, MAVLINK_MSG_ID_THREADTIME_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE THREADTIME UNPACKING


/**
 * @brief Get field time from threadtime message
 *
 * @return Time at which the message was sent
 */
static inline uint32_t mavlink_msg_threadtime_get_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field thread1 from threadtime message
 *
 * @return execution time of thread 1
 */
static inline uint32_t mavlink_msg_threadtime_get_thread1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field thread2 from threadtime message
 *
 * @return execution time of thread 2
 */
static inline uint32_t mavlink_msg_threadtime_get_thread2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field thread3 from threadtime message
 *
 * @return execution time of thread 3
 */
static inline uint32_t mavlink_msg_threadtime_get_thread3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field thread4 from threadtime message
 *
 * @return execution time of thread 4
 */
static inline uint32_t mavlink_msg_threadtime_get_thread4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field thread5 from threadtime message
 *
 * @return execution time of thread 5
 */
static inline uint32_t mavlink_msg_threadtime_get_thread5(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Get field thread6 from threadtime message
 *
 * @return execution time of thread 6
 */
static inline uint32_t mavlink_msg_threadtime_get_thread6(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  24);
}

/**
 * @brief Decode a threadtime message into a struct
 *
 * @param msg The message to decode
 * @param threadtime C-struct to decode the message contents into
 */
static inline void mavlink_msg_threadtime_decode(const mavlink_message_t* msg, mavlink_threadtime_t* threadtime)
{
#if MAVLINK_NEED_BYTE_SWAP
	threadtime->time = mavlink_msg_threadtime_get_time(msg);
	threadtime->thread1 = mavlink_msg_threadtime_get_thread1(msg);
	threadtime->thread2 = mavlink_msg_threadtime_get_thread2(msg);
	threadtime->thread3 = mavlink_msg_threadtime_get_thread3(msg);
	threadtime->thread4 = mavlink_msg_threadtime_get_thread4(msg);
	threadtime->thread5 = mavlink_msg_threadtime_get_thread5(msg);
	threadtime->thread6 = mavlink_msg_threadtime_get_thread6(msg);
#else
	memcpy(threadtime, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_THREADTIME_LEN);
#endif
}
