#ifndef OURBOT_HAL_H
#define OURBOT_HAL_H

#include "hal_base.h"
#include "sensor1D.h"
#include "encoder_sensor.h"
#include "pololuMC33926.h"
#include "analog_sensor.h"

class OurbotHAL : public HALBase
{
private:
    Sensor1D _battery;
    EncoderSensor _encoders[4];
    AnalogSensor _currents[4];

    PololuMC33926 _motors[4];

public:
	OurbotHAL();

    Sensor1D* getEncoder(int index);
    Sensor1D* getCurrentSensor(int index);
    HBridgeInterface* getMotor(int index);
};

#endif //OURBOT_HAL_H
