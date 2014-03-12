#include <EEPROM.h>

int EEPROM_read_int(int address)
{
  return int(  (((unsigned int)EEPROM.read(address)) << 8)  |  EEPROM.read(address + 1)  );
}

void EEPROM_write_int(int address, int value)
{
  EEPROM.write(address, ((unsigned int)value) >> 8);
  EEPROM.write(address + 1, value & 0xFF);
}

int serial_putc( char c, FILE * ) 
{
  Serial.write( c );
  return c;
} 
void printf_begin(void)
{
  fdevopen( &serial_putc, 0 );
}

