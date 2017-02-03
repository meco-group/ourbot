/** @file
 *	@brief MAVLink comm protocol testsuite generated from ourbot_messages.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef OURBOT_MESSAGES_TESTSUITE_H
#define OURBOT_MESSAGES_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_microos_messages(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_ourbot_messages(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_microos_messages(system_id, component_id, last_msg);
	mavlink_test_ourbot_messages(system_id, component_id, last_msg);
}
#endif

#include "../microos_messages/testsuite.h"


static void mavlink_test_motor_state(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_motor_state_t packet_in = {
		963497464,
	}963497672,
	}963497880,
	}963498088,
	}963498296,
	}18275,
	}18379,
	};
	mavlink_motor_state_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.position = packet_in.position;
        	packet1.velocity = packet_in.velocity;
        	packet1.acceleration = packet_in.acceleration;
        	packet1.current = packet_in.current;
        	packet1.reference = packet_in.reference;
        	packet1.FFvoltage = packet_in.FFvoltage;
        	packet1.FBvoltage = packet_in.FBvoltage;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motor_state_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_motor_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motor_state_pack(system_id, component_id, &msg , packet1.position , packet1.velocity , packet1.acceleration , packet1.current , packet1.FFvoltage , packet1.FBvoltage , packet1.reference );
	mavlink_msg_motor_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motor_state_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.position , packet1.velocity , packet1.acceleration , packet1.current , packet1.FFvoltage , packet1.FBvoltage , packet1.reference );
	mavlink_msg_motor_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_motor_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motor_state_send(MAVLINK_COMM_1 , packet1.position , packet1.velocity , packet1.acceleration , packet1.current , packet1.FFvoltage , packet1.FBvoltage , packet1.reference );
	mavlink_msg_motor_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_motor_command(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_motor_command_t packet_in = {
		963497464,
	}963497672,
	}963497880,
	}963498088,
	}53,
	};
	mavlink_motor_command_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.command_left_front = packet_in.command_left_front;
        	packet1.command_right_front = packet_in.command_right_front;
        	packet1.command_left_rear = packet_in.command_left_rear;
        	packet1.command_right_rear = packet_in.command_right_rear;
        	packet1.command_type = packet_in.command_type;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motor_command_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_motor_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motor_command_pack(system_id, component_id, &msg , packet1.command_type , packet1.command_left_front , packet1.command_right_front , packet1.command_left_rear , packet1.command_right_rear );
	mavlink_msg_motor_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motor_command_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.command_type , packet1.command_left_front , packet1.command_right_front , packet1.command_left_rear , packet1.command_right_rear );
	mavlink_msg_motor_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_motor_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motor_command_send(MAVLINK_COMM_1 , packet1.command_type , packet1.command_left_front , packet1.command_right_front , packet1.command_left_rear , packet1.command_right_rear );
	mavlink_msg_motor_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_motor_controller(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_motor_controller_t packet_in = {
		17.0,
	}45.0,
	}73.0,
	};
	mavlink_motor_controller_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.P = packet_in.P;
        	packet1.I = packet_in.I;
        	packet1.D = packet_in.D;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motor_controller_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_motor_controller_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motor_controller_pack(system_id, component_id, &msg , packet1.P , packet1.I , packet1.D );
	mavlink_msg_motor_controller_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motor_controller_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.P , packet1.I , packet1.D );
	mavlink_msg_motor_controller_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_motor_controller_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motor_controller_send(MAVLINK_COMM_1 , packet1.P , packet1.I , packet1.D );
	mavlink_msg_motor_controller_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_raw_imu_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_raw_imu_data_t packet_in = {
		{ 17235, 17236, 17237 },
	}{ 17547, 17548, 17549 },
	}{ 17859, 17860, 17861 },
	};
	mavlink_raw_imu_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.acc, packet_in.acc, sizeof(int16_t)*3);
        	mav_array_memcpy(packet1.gyro, packet_in.gyro, sizeof(int16_t)*3);
        	mav_array_memcpy(packet1.mag, packet_in.mag, sizeof(int16_t)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_raw_imu_data_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_raw_imu_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_raw_imu_data_pack(system_id, component_id, &msg , packet1.acc , packet1.gyro , packet1.mag );
	mavlink_msg_raw_imu_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_raw_imu_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.acc , packet1.gyro , packet1.mag );
	mavlink_msg_raw_imu_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_raw_imu_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_raw_imu_data_send(MAVLINK_COMM_1 , packet1.acc , packet1.gyro , packet1.mag );
	mavlink_msg_raw_imu_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ourbot_messages(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_motor_state(system_id, component_id, last_msg);
	mavlink_test_motor_command(system_id, component_id, last_msg);
	mavlink_test_motor_controller(system_id, component_id, last_msg);
	mavlink_test_raw_imu_data(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // OURBOT_MESSAGES_TESTSUITE_H
