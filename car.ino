/*
 * Car controller main entry point
 *
 * Project works on Arduino Mega 2560 only!
 */

// Arduino (GCC-AVR)
// Assumes that: char = 1 byte, int = 2 bytes, long = 4 bytes

#include <Wire.h>
#include <Serial.h>

struct { void(*loop_func)(); const char* name; } programs[] = {
  {distance_loop,  "distance drive"},
  {compas_loop,  "compas readout"},
  {twitchy_loop, "twitchy simple demo"}
};
void(*current_loop_func)();

void setup() {
  // reset motor controller, ensure it misses the servo pulse and puts itself in error mode
  delay(200);

  Serial.begin(57600);
  Serial.print("Car ");
  
  drivetrain_setup();
  lights_setup();
  distance_setup();
  // i2c and digital compas
  Wire.begin();
  compas_setup();
  // SPI and radio
  // SPI.begin();
  // radio_setup();

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
  delay(1500);
  int prog = Serial.parseInt();
  if (prog < 0 || prog >= (sizeof(programs)/sizeof(*programs)))
    prog = 0;
  current_loop_func = programs[prog].loop_func;
  Serial.print("Invoking program #");
  Serial.print(prog);
  Serial.print(": ");
  Serial.println(programs[prog].name);
  delay(1000);
}

void distance_loop()
{
  delay(10);
  
  unsigned char d = distance_forward();
  
  static unsigned char compas_delay = 0;
  if ((compas_delay++ & 0xf) == 0) //period
    compas_read();
  
  Serial.print("Distance: ~");
  Serial.print(d);
  Serial.print(" cm (<-> ");
  Serial.print(distance_raw_distance());
  Serial.print(" / ");
  Serial.print(distance_raw_raising());
  Serial.print(" \\ ");
  Serial.print(distance_raw_falling());
  Serial.print(" pins=");
  Serial.print(distance_raw_portstate());
  Serial.print(") Compas: ");
  Serial.print((int)compas_get_heading());
  Serial.print(" [");
  Serial.print(compas_get_x());
  Serial.print("; ");
  Serial.print(compas_get_y());
  Serial.print("; ");
  Serial.print(compas_get_z());
  Serial.println("]");

  const unsigned char T00 = 20;
  const unsigned char T0 = 25;
  const unsigned char T1 = 40;
  const unsigned char T2 = 80;
  
  if (d < T00)
    drivetrain_set_power(-60);
  if (d < T0)
    drivetrain_set_power(-50);
  else if (d < T1)
    drivetrain_set_power(0);
  else if (d < T2)
    drivetrain_set_power(50);
  else
    drivetrain_set_power(55);
    
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
    lights_set_headlight(30);  
  else
    lights_set_headlight(128);
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
    
    // TODO: Put this in lights_..
    if (++indicatorDelay >= 17) //every 1/3 sec
    {  
      indicatorDelay = 0;
      if (indicatorState == 0 || indicatorState == 0x40)
        indicatorState = 0x01;
      else if (indicatorState == 0x04)
        indicatorState = 0x10;
      else
        indicatorState <<= 1;
      PORTA = indicatorState;
    }
    
    Serial.print("Distance: ~");
    Serial.print(distance_forward());
    Serial.println(" cm");
}

void compas_loop()
{
  delay(1000/30);
  
  compas_read();
  
  Serial.print("Compas: ");
  Serial.print((int)compas_get_heading());
  Serial.print(" [");
  Serial.print(compas_get_x());
  Serial.print("; ");
  Serial.print(compas_get_y());
  Serial.print("; ");
  Serial.print(compas_get_z());
  Serial.println("]");
}

void loop()
{
  current_loop_func();
}

