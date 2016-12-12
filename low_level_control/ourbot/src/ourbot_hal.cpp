#include "ourbot_hal.h"

#define HBRIDGE0_ENCA_PIN   0
#define HBRIDGE0_ENCB_PIN   1
#define HBRIDGE0_ENC_ID     10
#define HBRIDGE1_ENCA_PIN   7 
#define HBRIDGE1_ENCB_PIN   8
#define HBRIDGE1_ENC_ID     11
#define HBRIDGE2_ENCA_PIN   19
#define HBRIDGE2_ENCB_PIN   18
#define HBRIDGE2_ENC_ID     12
#define HBRIDGE3_ENCA_PIN   17
#define HBRIDGE3_ENCB_PIN   16
#define HBRIDGE3_ENC_ID     13

#define HBRIDGE0_CUR_PIN    A15
#define HBRIDGE0_CUR_ID     20
#define HBRIDGE1_CUR_PIN    A16
#define HBRIDGE1_CUR_ID     21
#define HBRIDGE2_CUR_PIN    A17
#define HBRIDGE2_CUR_ID     22
#define HBRIDGE3_CUR_PIN    A18
#define HBRIDGE3_CUR_ID     23

#define HBRIDGE0_IN1_PIN    3
#define HBRIDGE0_IN2_PIN    4
#define HBRIDGE0_COMP_ID    0
#define HBRIDGE1_IN1_PIN    5
#define HBRIDGE1_IN2_PIN    6 
#define HBRIDGE1_COMP_ID    1
#define HBRIDGE2_IN1_PIN    23
#define HBRIDGE2_IN2_PIN    22
#define HBRIDGE2_COMP_ID    2
#define HBRIDGE3_IN1_PIN    21
#define HBRIDGE3_IN2_PIN    20
#define HBRIDGE3_COMP_ID    3

#define BATTERY_COMP_ID     30

OurbotHAL::OurbotHAL() :
	HALBase(),
	_battery(BATTERY_COMP_ID),
	_encoders{EncoderSensor(HBRIDGE0_ENCA_PIN, HBRIDGE0_ENCB_PIN, HBRIDGE0_ENC_ID),EncoderSensor(HBRIDGE1_ENCA_PIN, HBRIDGE1_ENCB_PIN, HBRIDGE1_ENC_ID), EncoderSensor(HBRIDGE2_ENCA_PIN, HBRIDGE2_ENCB_PIN, HBRIDGE2_ENC_ID), EncoderSensor(HBRIDGE3_ENCA_PIN, HBRIDGE3_ENCB_PIN, HBRIDGE3_ENC_ID)},
    _currents{AnalogSensor(HBRIDGE0_CUR_PIN, HBRIDGE0_CUR_ID), AnalogSensor(HBRIDGE1_CUR_PIN, HBRIDGE1_CUR_ID), AnalogSensor(HBRIDGE2_CUR_PIN, HBRIDGE2_CUR_ID), AnalogSensor(HBRIDGE3_CUR_PIN, HBRIDGE3_CUR_ID)},
	_motors{PololuMC33926(HBRIDGE0_IN1_PIN, HBRIDGE0_IN2_PIN, &_battery, HBRIDGE0_COMP_ID), PololuMC33926(HBRIDGE1_IN1_PIN, HBRIDGE1_IN2_PIN, &_battery, HBRIDGE1_COMP_ID), PololuMC33926(HBRIDGE2_IN1_PIN, HBRIDGE2_IN2_PIN, &_battery, HBRIDGE2_COMP_ID), PololuMC33926(HBRIDGE3_IN1_PIN, HBRIDGE3_IN2_PIN, &_battery, HBRIDGE3_COMP_ID)}
{	
    //virtual battery
	_battery.setRawValue(24000);
	_battery.setCalibratedValue(24000);
}

Sensor1D* OurbotHAL::getEncoder(int index)
{
    return &_encoders[index];
}

Sensor1D* OurbotHAL::getCurrentSensor(int index)
{
    return &_currents[index];
}

HBridgeInterface* OurbotHAL::getMotor(int index)
{
    return &_motors[index];
}
