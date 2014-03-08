/*
 * Car controller main entry point
 *
 * Project works on Arduino Mega 2560 only!
 */

// Arduino (GCC-AVR)
// Assumes that: char = 1 byte, int = 2 bytes, long = 4 bytes

#include <Serial.h>

void setup() {
  // reset motor controller, ensure it misses the servo pulse and puts itself in error mode
  delay(200);
  
  drivetrain_setup();
  lights_setup();
  distance_setup();

  Serial.begin(57600);
  Serial.println("Car controller, welcome!");

  // stabilize motor controller, sync up with pulse
  delay(3000);
}

void loop()
{
  delay(10);
  
  unsigned char d = distance_forward();
  
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
  Serial.println(")");

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

