#ifndef HAL_CONFIG_H
#define HAL_CONFIG_H

/*
	HALInterface configuration parameters
*/
#define HAL_INTERFACE_HBRIDGE_ENABLE	4
#define HAL_INTERFACE_STEPPER_ENABLE	0
#define HAL_INTERFACE_LED_ENABLE		1
#define HAL_INTERFACE_SENSOR_ENABLE		9
#define HAL_INTERFACE_IMU_ENABLE		0

#define HAL_INTERFACE_BATTERY_ENABLE
#define HAL_INTERFACE_AD_RESOLUTION	12	//Is only supported by Arduino Due and teensy, so don't enable it for other boards

#define OURBOT_HAL_V1

/*
 * Hardware version 0, better known as wires all over the place!
 */
#ifdef OURBOT_HAL_V0

	// Motor0 setup
	#define HBRIDGE0_COMP_ID	0		//component id
	#define HBRIDGE0_IN1_PIN	3		//direction pin for H-bridge - used in single and dual pin operation
	#define HBRIDGE0_IN2_PIN	4		//secondary direction pin for H-bridge - used only in dual pin operation
	#define HBRIDGE0_ENC_ID	10		//encoder hardware ID
	#define HBRIDGE0_ENCA_PIN	7		//encoder channel A pin
	#define HBRIDGE0_ENCB_PIN	8		//encoder channel B pin
	#define HBRIDGE0_CUR_ID	20		//current sensor hardware ID
	#define HBRIDGE0_CUR_PIN	A10		//current sensor pin

	// Motor1 setup
	#define HBRIDGE1_COMP_ID	1		//component id
	#define HBRIDGE1_IN1_PIN	5		//direction pin for H-bridge - used in single and dual pin operation
	#define HBRIDGE1_IN2_PIN	6		//secondary direction pin for H-bridge - used only in dual pin operation
	#define HBRIDGE1_ENC_ID	11
	#define HBRIDGE1_ENCA_PIN	9		//encoder channel A pin
	#define HBRIDGE1_ENCB_PIN	10		//encoder channel B pin
	#define HBRIDGE1_CUR_ID	21		//current sensor hardware ID
	#define HBRIDGE1_CUR_PIN	A11		//current sensor pin

	// Motor2 setup
	#define HBRIDGE2_COMP_ID	2		//component id
	#define HBRIDGE2_IN1_PIN	23		//direction pin for H-bridge - used in single and dual pin operation
	#define HBRIDGE2_IN2_PIN	22		//secondary direction pin for H-bridge - used only in dual pin operation
	#define HBRIDGE2_ENC_ID	12		
	#define HBRIDGE2_ENCA_PIN	19		//encoder channel A pin
	#define HBRIDGE2_ENCB_PIN	18		//encoder channel B pin
	#define HBRIDGE2_CUR_ID	22		//current sensor hardware ID
	#define HBRIDGE2_CUR_PIN	A12		//current sensor pin

	// Motor3 setup
	#define HBRIDGE3_COMP_ID	3		//component id
	#define HBRIDGE3_IN1_PIN	21		//direction pin for H-bridge - used in single and dual pin operation
	#define HBRIDGE3_IN2_PIN	20		//secondary direction pin for H-bridge - used only in dual pin operation
	#define HBRIDGE3_ENC_ID	13
	#define HBRIDGE3_ENCA_PIN	17		//encoder channel A pin
	#define HBRIDGE3_ENCB_PIN	16		//encoder channel B pin
	#define HBRIDGE3_CUR_ID	23		//current sensor hardware ID
	#define HBRIDGE3_CUR_PIN	A13		//current sensor pin

/*
 * Hardware version 1: first pcb design for ourbot.
 */
#elif defined(OURBOT_HAL_V1)

	// Motor0 setup - HB3
	#define HBRIDGE0_COMP_ID	0		//component id
	#define HBRIDGE0_IN1_PIN	21		//direction pin for H-bridge - used in single and dual pin operation
	#define HBRIDGE0_IN2_PIN	20		//secondary direction pin for H-bridge - used only in dual pin operation
	#define HBRIDGE0_ENC_ID	10		//encoder hardware ID
	#define HBRIDGE0_ENCA_PIN	17		//encoder channel A pin
	#define HBRIDGE0_ENCB_PIN	16		//encoder channel B pin
	#define HBRIDGE0_CUR_ID	20		//current sensor hardware ID
	#define HBRIDGE0_CUR_PIN	A13		//current sensor pin

	// Motor1 setup - HB1
	#define HBRIDGE1_COMP_ID	1		//component id
	#define HBRIDGE1_IN1_PIN	5		//direction pin for H-bridge - used in single and dual pin operation
	#define HBRIDGE1_IN2_PIN	6		//secondary direction pin for H-bridge - used only in dual pin operation
	#define HBRIDGE1_ENC_ID	11
	#define HBRIDGE1_ENCA_PIN	8		//encoder channel A pin
	#define HBRIDGE1_ENCB_PIN	7		//encoder channel B pin
	#define HBRIDGE1_CUR_ID	21		//current sensor hardware ID
	#define HBRIDGE1_CUR_PIN	A11		//current sensor pin

	// Motor2 setup - HB2
	#define HBRIDGE2_COMP_ID	2		//component id
	#define HBRIDGE2_IN1_PIN	23		//direction pin for H-bridge - used in single and dual pin operation
	#define HBRIDGE2_IN2_PIN	22		//secondary direction pin for H-bridge - used only in dual pin operation
	#define HBRIDGE2_ENC_ID	12		
	#define HBRIDGE2_ENCA_PIN	19		//encoder channel A pin
	#define HBRIDGE2_ENCB_PIN	18		//encoder channel B pin
	#define HBRIDGE2_CUR_ID	22		//current sensor hardware ID
	#define HBRIDGE2_CUR_PIN	A12		//current sensor pin

	// Motor3 setup - HB0
	#define HBRIDGE3_COMP_ID	3		//component id
	#define HBRIDGE3_IN1_PIN	3		//direction pin for H-bridge - used in single and dual pin operation
	#define HBRIDGE3_IN2_PIN	4		//secondary direction pin for H-bridge - used only in dual pin operation
	#define HBRIDGE3_ENC_ID	13
	#define HBRIDGE3_ENCA_PIN	1		//encoder channel A pin
	#define HBRIDGE3_ENCB_PIN	0		//encoder channel B pin
	#define HBRIDGE3_CUR_ID	23		//current sensor hardware ID
	#define HBRIDGE3_CUR_PIN	A10		//current sensor pin
#else
	#error No ourbot hardware version defined! Cannot compile.
#endif

/*
	Stepper motor setup
	STEPPER_TYPE: 1 = classic HBridge stepper with additional defines PIN1 - PIN4
				  2 = easyDriver stepper with additional defines PIN_DIR and PIN_STEP
*/
// Stepper0 setup
/*#define STEPPER0_ENABLE
#define STEPPER0_COMP_ID	20
#define STEPPER0_TYPE		2	
#define STEPPER0_PIN1		5	
#define STEPPER0_PIN2		6	
#define STEPPER0_PIN3		4	
#define STEPPER0_PIN4		3	
#define STEPPER0_DIR_PIN	3
#define STEPPER0_STEP_PIN	2*/


/*
	LED setup
*/
#define ONBOARDLED_COMP_ID		30
#define ONBOARDLED_PIN			13
/*#define LED0_PIN_COMP_ID		31
#define LED0_PIN		0
#define LED1_PIN_COMP_ID		32
#define LED1_PIN		0
#define LED2_PIN_COMP_ID		33
#define LED2_PIN		0
#define LED3_PIN_COMP_ID		34
#define LED3_PIN		0*/


/*
	IMU SETUP
*/
// IMU0 setup
//#define IMU0_COMP_ID	40


/*
	Analog Sensor setups
*/
//Battery setup
#define BATTERY_COMP_ID	50
#define BATTERY_PIN		A14
#define BATTERY_SCALE	3.4305f //(4.257*3.3/(2^12))

#endif //HAL_CONFIG_H
