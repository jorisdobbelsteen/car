/*
 * Lights controller
 *
 * API:
 *   void lights_setup()
 *       Initializes lights module
 *   void lights_set_headlight(unsigned char intensity)
 *   void lights_set_headlight(unsigned char left, unsigned char right)
 *       Set intensity of headlights between 0 (off) and 255 (fully on). Can be set independently, but convienience function is provided to set both at once.
 *   void lights_set_rearlight(unsigned char intensity)
 *       Set intensity of rear lights between 0 (off) and 255 (fully on).
 *       At 255 they are as intense as the break light, so recommended values are 0 for off, 80 for "normal" rearlight and 255 when "combining" with breaklight.
 *   void lights_set_breaklight_on()
 *   void lights_set_breaklight_off()
 *       Turn breaklights on and off.
 *   void lights_set_indicator_off()
 *   void lights_set_indicator_left()
 *   void lights_set_indicator_right()
 *   void lights_set_indicator_both()
 *   void lights_set_indicator(unsigned char left, unsigned char right)
 *   void lights_set_indicator_mask(unsigned char mask)
 *       Turn on indicator lights in blinking mode
 *   void lights_set_indicator_override_mask(unsigned char mask)
 *       Turn on indicator turned on fully
 */

static unsigned char m_lights_indicator;
static unsigned char m_lights_indicator_override;
 
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
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
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

  // turn off indicators
  m_lights_indicator = 0;
  m_lights_indicator_override = 0;
  // Enable timer interrupt
  TIMSK4 = _BV(TOIE4);

  // breaklight
  pinMode(4, OUTPUT);
  lights_set_breaklight_off();
}

void lights_set_headlight(unsigned char intensity) { lights_set_headlight(intensity, intensity); }
void lights_set_headlight(unsigned char left, unsigned char right)
{
  OCR4A = left;
  pinMode(6, (left == 0) ? INPUT : OUTPUT);
  OCR4B = right;
  pinMode(7, (right == 0) ? INPUT : OUTPUT);
}

void lights_set_rearlight(unsigned char intensity)
{
  OCR4C = intensity;
  pinMode(8, (intensity == 0) ? INPUT : OUTPUT);
}

void lights_set_breaklight_on()  { digitalWrite(4, HIGH); }
void lights_set_breaklight_off() { digitalWrite(4, LOW);  }

unsigned char m_lights_isr_prescaler;
unsigned char m_lights_isr_count;
// Timer4 overflow, at 1KHz.. Needs to be fast not to interfere with distance software!
// We should piggyback on the 61 Hz drivetrain timer instead???
ISR(TIMER4_OVF_vect)
{
  if((++m_lights_isr_prescaler & 0x3f) == 0)
  {
    // 16 Hz indicator speeds.
    if (m_lights_indicator != 0)
    {
      PORTA = ((++m_lights_isr_count & (1<<2)) ? 0 : m_lights_indicator) | m_lights_indicator_override;
    }
    else
    {
      PORTA = m_lights_indicator_override;
      m_lights_isr_count = 0;
    }
  }
}

void lights_set_indicator_off()                                    { m_lights_indicator = 0; }
void lights_set_indicator_left()                                   { m_lights_indicator = 0xf0; }
void lights_set_indicator_right()                                  { m_lights_indicator = 0x0f; }
void lights_set_indicator_both()                                   { m_lights_indicator = 0xff; }
void lights_set_indicator(unsigned char left, unsigned char right) { m_lights_indicator = (left ? 0xf0 : 0) | (right ? 0x0f : 0); }
void lights_set_indicator_mask(unsigned char mask)                 { m_lights_indicator = mask; }

void lights_set_indicator_override_mask(unsigned char mask)        { m_lights_indicator_override = mask; }

