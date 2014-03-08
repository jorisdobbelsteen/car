/*
 * Lights controller
 */
 
void lights_setup()
{
  // Configure light outputs
  // Timer 3
  // WGM3:0 = mode 5 (fast PWM 8-bit)
  // CS[2:0] = 3, prescaler div 64
  //
  //       Clock  / prescaler / counter TOP = PWM period
  //       16 MHz / 64       / 256 (ICR3) = 976 Hz
  //          Note: LEDs require >100Hz to ensure human eye does not pick up on PWM.
  //
  //   Pin 6 left headlight  = Timer Output A
  //   Pin 7 right headlight = Timer Output B
  //   Pin 8 rearlights      = Timer Output C
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1) | _BV(WGM40); // COMnx[1:0] = clear on compare, WGM[1:0] see up
  TCCR4B = _BV(WGM42) | _BV(CS41) | _BV(CS40); // WGM[3:2] and CS[2:0] see up
  TCCR4C = 0; // FOCnx = 0
  lights_set_headlight(0);
  lights_set_rearlight(0);

  // Indicator lights left, port A
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT);
  // Indicator lights right, port A
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(29, OUTPUT);

  // breaklight
  pinMode(4, OUTPUT);
  lights_set_breaklight_off();
}

void lights_set_headlight(unsigned char intensity) { lights_set_headlight(intensity, intensity); }
void lights_set_headlight(unsigned char left, unsigned char right)
{
  OCR4A = left;
  OCR4B = right;
}

void lights_set_rearlight(unsigned char intensity)
{
  OCR4C = intensity;
}

void lights_set_breaklight_on()  { digitalWrite(4, HIGH); }
void lights_set_breaklight_off() { digitalWrite(4, LOW);  }


