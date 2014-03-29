#include <EEPROM.h>

/*
 * Battery voltage readout
 */
static unsigned int batt_last_millivolt; // filtering (dampening measurement)
void batt_setup()
{
  batt_last_millivolt = analogRead(A0); // initialize
}
unsigned int batt_get_millivolt()
{
  // 0 = 0V; 1023 = 10V (5 volt, but we use a voltage divider /2)
  // However on servo it's 1023 = 9.6V (power is 4.8V)
  
  // Read value from ADC and average with recent data
  unsigned int newvalue = analogRead(A0);
  newvalue += batt_last_millivolt * 31;
  newvalue /= 32;
  batt_last_millivolt = newvalue;
  
  // Output in millivolts
  newvalue *= 10;
  return newvalue - (newvalue >> 6); // value correction
}

/*
 * Safety signal
 */
void utils_setup()
{
  pinMode(12,INPUT_PULLUP);
}
bool utils_program_enable()
{
  return digitalRead(12) == LOW;
}

/*
 * sort function
 */
void utils_sort(unsigned char* a, unsigned char n)
{
  for (unsigned char i = 1; i < n; ++i)
  {
    unsigned char j = a[i];
    unsigned char k;
    for (k = i; (k > 0) && (j < a[k-1]); k--)
    {
      a[k] = a[k-1];
    }
    a[k] = j;
  }
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

