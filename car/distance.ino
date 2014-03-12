/*
 * 5-channel HC-SR04 ultrasound distance sensor module.
 */

static unsigned char m_portstate;
static unsigned int  m_raising, m_falling;
static unsigned int  m_last_raising, m_last_falling;
static unsigned char m_distance_forward;
static unsigned char m_distance_forward_last;

void distance_setup()
{
  // Configure ultrasonic sensor timer for trigger
  // Timer 5
  //   WGM3:0 = mode 15 (PWM up to ICR5)
  // Pin 46 ultrasound module trigger (timer5 port A)
  // Pin 45 ultrasound module trigger (timer5 port B)
  // Pin 44 ultrasound module trigger (timer5 port C)
  pinMode(46, OUTPUT);
  pinMode(45, OUTPUT);
  pinMode(44, OUTPUT);
  //
  // Ultrasound_accuracy describes the number of mm every counter tick. The ultrasound pulse is send, bounced back and received.
  // A pulse of 58us means a object is 1cm away.
  // Sensor gives pulse of 35 ms when nothing is detected. Time between pulses of about 60ms is recommended in the documentation,
  // which might be rather conservative.
  //
  //  Prescaler  Step_Time  Period_Time@16-bit   Ultrasound_accuracy*
  //    /1        62.5ns       ~4ms     Unsuitable! Period too short!
  //    /8         0.5us       32.7ms   Unsuitable! Period just too short!
  //    /64        4.0us      262.1ms                     ~0.7 mm          <<-- used
  // CS[2:0] = 3: /64 prescaler
  //
  //  Period = 50 ms
  //     60ms / 4us = 60*1000/4 = 60*250 = 15000
  ICR5 = (80*250);
  // Fist pulse should be 10us
  OCR5A = 3;                              // 12 us <<--------------- ISSUE!!!
  OCR5B = 3;                              // 12 us <<--------------- ISSUE!!!
  OCR5C = 3;                              // 12 us <<--------------- ISSUE!!!
  // Configure timer with above settings
  TCCR5A = _BV(COM5A1) | _BV(COM5B1) | _BV(COM5C1) | _BV(WGM51); // COMnx[1:0] = clear on compare, WGM[1:0] see up
  TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS51) | _BV(CS50); // WGM[3:2] and CS[2:0] see up
  TCCR5C = 0; // FOCnx = 0
  
  // PinChange pins 11 to 15 (map to PortK[3:7] on Mega2560)
  pinMode(11, INPUT);
  pinMode(12, INPUT);
  pinMode(13, INPUT);
  pinMode(14, INPUT);
  pinMode(15, INPUT);

  // Enable timer interrupt
  TIMSK5 = _BV(TOIE5);
  
  // Enable PinChange2 interrupt on the right pins
  PCMSK2 = 0x08; // only PortK[3]
  PCICR |= _BV(PCIE2); // enable PCINT2
}

// Returns approximate centimers to target. Value 255 is used for "clear" path.
// Returns 0 for sensor erros
unsigned char distance_forward() { return m_distance_forward; }
unsigned char distance_forward_left() { return 0; }
unsigned char distance_forward_right() { return 0; }
unsigned char distance_rear_left() { return 0; }
unsigned char distance_rear_right()  { return 0; }

ISR(PCINT2_vect)
{
  unsigned int port = PINK & (1<<3);
  if (port != m_portstate)
  {
    if(port)
      m_raising = TCNT5;
    else
      m_falling = TCNT5;
  }
  m_portstate = port;
}

ISR(TIMER5_OVF_vect)
{
  // 1 counter tick = 4us. Every 58us is 1cm
  // so divide by 58 / 4 = 14.5
  // for fast approximation we use 16 instead as a divider, since it can be done by shifts...
  
  // expect raising before falling
  if (m_raising != 0 && m_falling != 0) // if invalid, skip them...
  {
    m_distance_forward_last = m_distance_forward;
    unsigned int distance = m_falling - m_raising;
    if (distance > (16 * 255))
      m_distance_forward = 255;
    else
      m_distance_forward = distance / 16;
  }
  else
  {
    // sensor error...
  }
  m_last_raising = m_raising;
  m_last_falling = m_falling;
  m_raising = 0;
  m_falling = 0;
}

unsigned int distance_raw_raising()
{
  return m_last_raising;
}
unsigned int distance_raw_falling()
{
  return m_last_falling;
}
unsigned int distance_raw_distance()
{
  return m_last_falling - m_last_raising;
}
unsigned char distance_raw_portstate()
{
  return m_portstate;
}

