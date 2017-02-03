#ifndef EEPROM_M_H
#define EEPROM_M_H

//#include "WProgram.h"
#include "EEPROM.h"
#include "eeprom_m_variables.h"
	
class EEPROM_M
{
private:
	static const bool _init;
	static uint16_t _index[NUMBER_ENTRIES+2];	

public:
/*
 * Definition of variable access functions. Each variable receives her own access function: type name(), her own modifier function type set_name(type name), an inline index accessing function and a noinline version for convenience when debugging. 
 */
	#define ENTRY(name,type)	static type name();	\
								static type set_##name(type name); \
								inline static uint16_t inline_##name##_index(); \
								static uint16_t name##_index();
	EEPROM_TABLE
	#undef ENTRY

	static bool init(){
		for(uint8_t k=0;k<(NUMBER_ENTRIES);k++){ _index[k+1] += _index[k]; }
		return true;
	}
};

#endif //EEPROM_M_H
