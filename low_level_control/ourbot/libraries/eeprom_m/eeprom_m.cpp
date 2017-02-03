#include "eeprom_m.h"

uint16_t EEPROM_M::_index[] = { EEPROM_INDEX, 
#define ENTRY(name,type)	sizeof(type),					
EEPROM_TABLE
#undef ENTRY
0 };

const bool EEPROM_M::_init = EEPROM_M::init();

#define ENTRY(name,type) \
inline uint16_t EEPROM_M::inline_##name##_index(){ \
	return _index[__COUNTER__]; \
}\ 
\
uint16_t EEPROM_M::name##_index(){ \
	return inline_##name##_index(); \
}	
EEPROM_TABLE
#undef ENTRY

#define ENTRY(name,type) \
type EEPROM_M::name(){ \
	type name; \
	char* p = (char*)&name; \
	unsigned int i; \
	int ee = inline_##name##_index(); \
	for (i = 0; i < sizeof(name); i++){ \
		*p = EEPROM.read(ee); \
		*p++; ee++; \
	} \
	return name; \
}	 
EEPROM_TABLE
#undef ENTRY

#define ENTRY(name,type) \
type EEPROM_M::set_##name(type name){ \
	char* p = (char*)&name; \
	unsigned int i; \
	int ee = inline_##name##_index(); \
	for (i = 0; i < sizeof(name); i++){ \
		EEPROM.write(ee, *p); \
		*p++; ee++; \
	} \
	return name; \
}	 
EEPROM_TABLE
#undef ENTRY
