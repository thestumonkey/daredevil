#include "debug.h"
#include "defs.h"
#include "debug_defs.h"

#define __PROG_TYPES_COMPAT__
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <SPI.h>

char line[ 24 ];

int freeRAM() 
{
	extern int __heap_start, *__brkval; 
	int v; 
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void dumpSettings()
{
	strncpy_P( line, (char*)pgm_read_word(&(DBG_STRS[ 0 ])), 24 );
	Serial.println( line );

	strncpy_P( line, (char*)pgm_read_word(&(DBG_STRS[ 1 ])), 24 );

	// Auto cal values
	Serial.print( line );
	Serial.println( EEPROM.read( MOTOR_VALS_ADDR ), HEX );
}

void printFirmwareInfo()
{
	for( uint8_t i = 2 ; i < 4 ; ++i )
	{
		strncpy_P( line, (char*)pgm_read_word(&(DBG_STRS[ i ])), 24 );
		Serial.println( line );
	}
}


