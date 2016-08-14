#ifndef diuEEPROM
#define diuEEPROM

#include <avr/eeprom.h>

template <class T> int EEPROM_writeData (int ee, const T& value)
{
    eeprom_write_block((const void*)value, (void*)ee, sizeof(*value));
	return sizeof(value);
}

template <class T> int EEPROM_readData(int ee, T& value)
{
    eeprom_read_block((void*)value, (void*)0, sizeof(*value));
    return sizeof(*value);
}

/*
#include <E:\Electronic\Softwares\Arduino Setup\arduino-0023\libraries\EEPROM\EEPROM.h>
#include <avr/eeprom.h>
#include <Arduino.h>  // for type definitions


template <class T> int EEPROM_writeData(int ee, const T& value)
{
	EEPROMClass EEPROM;
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
		//eeprom_write_byte(ee++,*p++);
	    EEPROM.write(ee++, *p++);
    return i;
}
*/
/*
void EEPROM_writeData (const void *__dst, void *__src, size_t __n)
{
    eeprom_write_block(__dst, __src, __n);
}*/
/*
template <class T> int EEPROM_readData(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
		//*p++ = eeprom_read_byte(ee++);
	    *p++ = EEPROM.read(ee++);
    return i;
}
*/
#endif