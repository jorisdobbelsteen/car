#include <EEPROM.h>

/*
 * Battery voltage readout
 */
static unsigned int batt_last_millivolt = 7200; // filtering (dampening measurement)
unsigned int batt_get_millivolt()
{
  // 0 = 0V; 1023 = 10V (5 volt, but we use a voltage divider /2)
  // However on servo it's 1023 = 9.6V (power is 4.8V)
  unsigned int value = analogRead(A0);
  batt_last_millivolt = (batt_last_millivolt / 2) + (value * (10 / 2));
  return batt_last_millivolt - (batt_last_millivolt >> 6); // value correction
  //return value * 10;
}

/*
 * EEPROM helper routines
 */
int EEPROM_read_int(int address)
{
  return int(  (((unsigned int)EEPROM.read(address)) << 8)  |  EEPROM.read(address + 1)  );
}

void EEPROM_write_int(int address, int value)
{
  EEPROM.write(address, ((unsigned int)value) >> 8);
  EEPROM.write(address + 1, value & 0xFF);
}

/*
 * printf support via serial port (used by RF24)
 */
int serial_putc( char c, FILE * ) 
{
  Serial.write( c );
  return c;
} 
void printf_begin(void)
{
  fdevopen( &serial_putc, 0 );
}

