/*
 * Car controller main entry point
 *
 * Project works on Arduino Mega 2560 only!
 */

// Arduino (GCC-AVR)
// Assumes that: char = 1 byte, int = 2 bytes, long = 4 bytes

#include <Wire.h>
#include <Serial.h>
#include <SPI.h>

struct { void(*loop_func)(); void(*setup_func)(); const char* name; } programs[] = {
  {selfdriving_loop, selfdriving_setup, "self-driving"},
  {distance_loop, NULL, "distance driving"},
  {compas_calibration_loop, compas_calibration_setup, "compas calibration"},
  {twitchy_loop, NULL, "simple 'twitchy' demo"}
};
void(*current_loop_func)();

void setup() {
  // reset motor controller, ensure it misses the servo pulse and puts itself in error mode
  delay(200);

  Serial.begin(57600);
  printf_begin(); // RF24 uses this...
  Serial.print("Car ");
  
  drivetrain_setup();
  lights_setup();
  distance_setup();
  // i2c and digital compas
  Wire.begin();
  compas_setup();
  compas_calibration_load_from_eeprom();
  // SPI and radio
  SPI.begin();
  radio_setup();

  Serial.print("controller, ");

  // stabilize motor controller, sync up with pulse
  delay(500);
 
  Serial.println("welcome!");
 
  // print main programs and allow user to choose
  for(size_t i = 0; i < (sizeof(programs)/sizeof(*programs)); ++i)
  {
    Serial.print(i);
    Serial.print(") ");
    Serial.println(programs[i].name);
  }
  for (unsigned int i = 0; i < 10; ++i)
  {
    lights_set_rearlight((i & 0x1) ? 0 : 128);
    delay(150);
  }
  int prog = Serial.parseInt();
  if (prog < 0 || prog >= (sizeof(programs)/sizeof(*programs)))
    prog = 0;
  current_loop_func = programs[prog].loop_func;
  Serial.print("Invoking program #");
  Serial.print(prog);
  Serial.print(": ");
  Serial.println(programs[prog].name);
  if(programs[prog].setup_func)
    programs[prog].setup_func();
  delay(1000);
}
void loop()
{
  // handle modules
  radio_tick();
  // run selected program
  current_loop_func();
}

void distance_loop()
{
  delay(10);
  
  unsigned char d = distance_forward();
  
  static unsigned char compas_delay = 0;
  if ((compas_delay++ & 0xf) == 0) //period
    compas_read();
  
  static unsigned char print_delay = 0;
  if ((print_delay++ & 0x1f) == 0) //period
  {
    Serial.print("Distance: [c=");
    Serial.print(d);
    Serial.print(" l=");
    Serial.print(distance_forward_left());
    Serial.print(" r=");
    Serial.print(distance_forward_right());
    Serial.print("; l=");
    Serial.print(distance_rear_left());
    Serial.print(" r=");
    Serial.print(distance_rear_right());
    Serial.print(" cm Compas: ");
    Serial.print((int)compas_get_heading());
    Serial.print(" [");
    Serial.print(compas_get_x());
    Serial.print("; ");
    Serial.print(compas_get_y());
    Serial.print("; ");
    Serial.print(compas_get_z());
    Serial.print("] batt=");
    Serial.print(batt_get_millivolt());
    Serial.println("mV");

    if (0) // debug ultrasound
    {
      for(unsigned char i = 0; i < 12; ++i)
      {
        Serial.print("[");
        Serial.print((int)distance_debug_portstate(i));
        Serial.print(", ");
        Serial.print(distance_debug_timer(i));
        Serial.print("]");
      }
      Serial.println();
    }
  }

  const unsigned char T00 = 40;
  const unsigned char T0 = 45;
  const unsigned char T1 = 60;
  const unsigned char T2 = 80;
  
  if (d < T00)
    drivetrain_set_power(-60);
  if (d < T0)
    drivetrain_set_power(-57);
  else if (d < T1)
    drivetrain_set_power(0);
  else if (d < T2)
    drivetrain_set_power(57);
  else
    drivetrain_set_power(60);
    
  if(d < T0)
    lights_set_breaklight_on();
  else
    lights_set_breaklight_off();
    
  if(d < T0)
    lights_set_rearlight(255);
  if(d < T1)
    lights_set_rearlight(128);
  else
    lights_set_rearlight(25);

  if(d < T2)
    lights_set_headlight(10);
  else
    lights_set_headlight(50);
}

int dir = 0;
unsigned char indicatorDelay = 0;
unsigned char indicatorState = 0;
void twitchy_loop()
{
    delay(20);
    signed char currentspeed = drivetrain_get_power();
    if (dir == 0)
    {
      currentspeed++;
      if (currentspeed == 127)
        dir = 1;
    }
    else
    {
      currentspeed--;
      if (currentspeed == -127)
        dir = 0;
    }
    drivetrain_set_power(currentspeed);
    
    if(currentspeed > 10)
    {
        drivetrain_set_steer(127); // right
        lights_set_headlight(20,80);
        lights_set_rearlight(20);
        lights_set_breaklight_off();
    }
    else if (currentspeed < -10)
    {
        drivetrain_set_steer(-127); // left
        lights_set_headlight(80,20);
        lights_set_rearlight(255);
        lights_set_breaklight_on();
    }
    else
    {
        drivetrain_set_steer(0); // left
        lights_set_headlight(10);
        lights_set_rearlight(0);
        lights_set_breaklight_off();
    }
    
    if (++indicatorDelay >= 17) //every 1/3 sec
    {  
      indicatorDelay = 0;
      switch (indicatorState)
      {
        case 0x01: indicatorState = 0x04; break; // left front -> left rear
        case 0x04: indicatorState = 0x40; break; // left rear -> right rear
        case 0x40: indicatorState = 0x10; break; // right rear -> right front
        case 0x10: indicatorState = 0x01; break; // right front -> left front
        default: indicatorState = 0x01; break; 
      }
      lights_set_indicator_override_mask(indicatorState);
    }
    
    Serial.print("Distance: ~");
    Serial.print(distance_forward());
    Serial.println(" cm");
}

