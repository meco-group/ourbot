#include "hal_interface.h"

/*Macro to search component by ID*/
#define SEARCH_COMPONENT_MACRO(list,count) \
	uint8_t k=0; \
	while(k<count){ \
		if(list[k]->ID()==ID){ break; } \
		k++;\
	} \
	if(k>=count){ k=0; } \
	return list[k];

HALInterface::HALInterface(uint8_t ID) :
	#if HAL_INTERFACE_HBRIDGE_ENABLE > 0
		_hbridges(NULL), 
		_hbridge_count(HAL_INTERFACE_HBRIDGE_ENABLE), 
	#endif
	#if HAL_INTERFACE_STEPPER_ENABLE > 0
		_steppers(NULL), 
		_stepper_count(HAL_INTERFACE_STEPPER_ENABLE),
	#endif
	#if HAL_INTERFACE_LED_ENABLE > 0
		_leds(NULL), 
		_led_count(HAL_INTERFACE_LED_ENABLE),
	#endif
	#if HAL_INTERFACE_SENSOR_ENABLE > 0
		_sensors(NULL), 
		_sensor_count(HAL_INTERFACE_SENSOR_ENABLE),
	#endif
	#if HAL_INTERFACE_IMU_ENABLE > 0
		_imus(NULL), 
		_imu_count(HAL_INTERFACE_IMU_ENABLE),
	#endif
	ComponentInterface(ID)
{
	//do nothing
}

bool HALInterface::init()
{
	uint8_t k;
#if HAL_INTERFACE_HBRIDGE_ENABLE > 0
	for(k=0;k<HAL_INTERFACE_HBRIDGE_ENABLE;k++) _hbridges[k]->init();
#endif
#if HAL_INTERFACE_STEPPER_ENABLE > 0
	for(k=0;k<HAL_INTERFACE_STEPPER_ENABLE;k++) _steppers[k]->init();
#endif
#if HAL_INTERFACE_LED_ENABLE > 0
	for(k=0;k<HAL_INTERFACE_LED_ENABLE;k++) _leds[k]->init();
#endif
#if HAL_INTERFACE_SENSOR_ENABLE > 0
	for(k=0;k<HAL_INTERFACE_SENSOR_ENABLE;k++) _sensors[k]->init();
#endif
#if HAL_INTERFACE_IMU_ENABLE > 0
	for(k=0;k<HAL_INTERFACE_IMU_ENABLE;k++) _imus[k]->init();
#endif

#ifdef HAL_INTERFACE_AD_RESOLUTION
	analogReadResolution(HAL_INTERFACE_AD_RESOLUTION);
#endif
	return true;
}

#if HAL_INTERFACE_HBRIDGE_ENABLE > 0
HBridgeInterface* HALInterface::hbridgeID(uint8_t ID)
{
	SEARCH_COMPONENT_MACRO(_hbridges,_hbridge_count)
}

HBridgeInterface* HALInterface::getHBridge(uint8_t index)
{
	if(index<_hbridge_count) return _hbridges[index];
	else return _hbridges[0];
}
#endif

#if HAL_INTERFACE_STEPPER_ENABLE > 0
Stepper* HALInterface::stepperID(uint8_t ID)
{
	SEARCH_COMPONENT_MACRO(_steppers,_stepper_count)
}

Stepper* HALInterface::getStepper(uint8_t index)
{
	if(index<_stepper_count) return _steppers[index];
	else return _steppers[0];
}
#endif

#if HAL_INTERFACE_LED_ENABLE > 0
LED* HALInterface::ledID(uint8_t ID)
{
	SEARCH_COMPONENT_MACRO(_leds,_led_count)
}

LED* HALInterface::getLed(uint8_t index)
{
	if(index<_led_count) return _leds[index];
	else return _leds[0];
}

LED* HALInterface::onboardLed()
{
	return _leds[0];
}
#endif

#if HAL_INTERFACE_SENSOR_ENABLE > 0
Sensor1D* HALInterface::sensorID(uint8_t ID)
{
	SEARCH_COMPONENT_MACRO(_sensors,_sensor_count)
}

Sensor1D* HALInterface::getSensor(uint8_t index)
{
	if(index<_sensor_count) return _sensors[index];
	else return _sensors[0];
}
#endif

#if HAL_INTERFACE_IMU_ENABLE > 0
IMUInterface* HALInterface::imuID(uint8_t ID)
{
	SEARCH_COMPONENT_MACRO(_imus,_imu_count)
}

IMUInterface* HALInterface::getImu(uint8_t index)
{
	if(index<_imu_count) return _imus[index];
	else return _imus[0];
}
#endif

#ifdef HAL_INTERFACE_BATTERY_ENABLE
Sensor1D *HALInterface::batteryVoltageSensor()
{
	return _sensors[0];
}

int HALInterface::readBatteryVoltage()
{
	return this->batteryVoltageSensor()->readCalibratedValue();
}

int HALInterface::peekBatteryVoltage()
{
	return this->batteryVoltageSensor()->peekCalibratedValue();
}
#endif

