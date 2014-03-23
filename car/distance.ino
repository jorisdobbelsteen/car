/*
 * 5-channel HC-SR04 ultrasound distance sensor module.
 *
 * Event-driven (interrupt) distance sensor Sends out a ultrasonic pulse periodically.
 *
 * API:
 *  void distance_setup()
 *      Initializes the ultrasound sensors.
 *  unsigned char distance_forward()
 *  unsigned char distance_forward_left()
 *  unsigned char distance_forward_right()
 *  unsigned char distance_rear_left()
 *  unsigned char distance_rear_right()
 *      Indicates last measured distance for the sensor location in (approximate) centimeters. Returns
 *      zero when sensor error is detected.
 *  bool distance_updated()
 *      Indicates the distances have been updated since the last call to
 *      distance_updated function.
 */

static unsigned char m_distance[5]; // computed distance (every time overflow)
static unsigned char m_errors[5]; // keeps track of amount of errors.
static bool m_distance_updated;

static unsigned char m_pinchange_count;
static unsigned int m_pinchange_timer[12] = {0};
static unsigned char m_pinchange_portstate[12];
static unsigned char m_pinchange_lastportstate; // last pinchange portstate
//static unsigned char m_pinchange_overflow_lastportstate; // timer overflow pinchange portstate

unsigned char distance_debug_portstate(unsigned char i) { return m_pinchange_portstate[i]; }
unsigned int distance_debug_timer(unsigned char i)      { return m_pinchange_timer[i]; }

// 58us/cm --> 4m = 23ms. Essential is that the period allows echo's to die out. 
// Fast measurements allow faster car responses and more time to reject invalid data.
#define MEASUREMENT_PERIOD_MS 50
#define TICKS_PER_MS 250

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
  // which might be rather conservative. It is essential to have the pulse "die" out to prevent late echos from being detected and
  // interfering.
  //
  //  Prescaler  Step_Time  Period_Time@16-bit   Ultrasound_accuracy*
  //    /1        62.5ns       ~4ms     Unsuitable! Period too short!
  //    /8         0.5us       32.7ms   Unsuitable! Period just too short!
  //    /64        4.0us      262.1ms                     ~0.7 mm          <<-- used
  // CS[2:0] = 3: /64 prescaler
  //
  //  Measurement period
  //     example: 60ms / 4us = 60*1000/4 = 60*250 = 60*TICKS_PER_MS = 15000
  ICR5 = (MEASUREMENT_PERIOD_MS * TICKS_PER_MS);
  // Fist pulse should be 10us
  OCR5A = 3;                              // 12 us <<--------------- ISSUE!!!
  OCR5B = 3;                              // 12 us <<--------------- ISSUE!!!
  OCR5C = 3;                              // 12 us <<--------------- ISSUE!!!
  // Configure timer with above settings
  TCCR5A = _BV(COM5A1) | _BV(COM5B1) | _BV(COM5C1) | _BV(WGM51); // COMnx[1:0] = clear on compare, WGM[1:0] see up
  TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS51) | _BV(CS50); // WGM[3:2] and CS[2:0] see up
  TCCR5C = 0; // FOCnx = 0
  
  // PinChange pins 11 to 15 (map to PortK[3:7] on Mega2560)
  pinMode(A11, INPUT); //PortK[3]
  pinMode(A12, INPUT); //PortK[4]
  pinMode(A13, INPUT); //PortK[5]
  pinMode(A14, INPUT); //PortK[6]
  pinMode(A15, INPUT); //PortK[7]

  // Enable timer interrupt
  TIMSK5 = _BV(TOIE5);
  
  // Enable PinChange2 interrupt on the right pins
  PCMSK2 = 0xf8;       // on PortK[7:3]
  PCICR |= _BV(PCIE2); // enable PCINT2
  
  // reset
  m_pinchange_count = 0;
  m_pinchange_lastportstate = 0;
  // reset distances to error
  m_distance[0] = m_distance[1] = m_distance[2] = m_distance[3] = m_distance[4] = 0;
}

// Returns approximate centimers to target. Value 255 is used for "clear" path.
// Returns 0 for sensor erros
unsigned char distance_forward() { return m_distance[2]; }
unsigned char distance_forward_left() { return m_distance[0]; }
unsigned char distance_forward_right() { return m_distance[1]; }
unsigned char distance_rear_left() { return m_distance[4]; }
unsigned char distance_rear_right()  { return m_distance[3]; }
bool distance_updated() { bool res = m_distance_updated; m_distance_updated = false; return res; }

ISR(PCINT2_vect)
{
  unsigned char port = PINK & 0xf8;
  if (port == m_pinchange_lastportstate) // filter spurious interrupts
    return;
    
  if (m_pinchange_count < 12)
  {
    m_pinchange_timer[m_pinchange_count] = TCNT5;
    m_pinchange_portstate[m_pinchange_count] = port;
    ++m_pinchange_count;
  }
  
  m_pinchange_lastportstate = port;
}

ISR(TIMER5_OVF_vect)
{
  // 1 counter tick = 4us. Every 58us is 1cm
  // so divide by 58 / 4 = 14.5
  // for fast approximation we use 16 instead as a divider, since it can be done by shifts...

  unsigned int raising[5] = {0};
  unsigned int falling[5] = {0};

//  unsigned char port = m_pinchange_overflow_lastportstate;
//  if (m_pinchange_count > 0)
//    m_pinchange_overflow_lastportstate = m_pinchange_portstate[m_pinchange_count - 1];
  unsigned char port = 0;
  
  // process pinchange list for raising and falling times
  for(unsigned char i = 0; i < m_pinchange_count; ++i)
  {
    unsigned char newport = m_pinchange_portstate[i];
    unsigned char pinchange = port ^ newport; // xor becomes 1 when value changes
    for (unsigned char j = 0, pinmask = (1<<3); j < 5; ++j, pinmask<<=1)
    {
      if (pinchange & pinmask)
      {
        if (newport & pinmask)
          raising[j] = m_pinchange_timer[i];
        else
          falling[j] = m_pinchange_timer[i];
      }
    }
    m_pinchange_timer[i] = 0;
    port = newport;
  }
  
  // Process collected data for up and down changes
  for (unsigned char i = 0, pinmask = (1<<3); i < 5; ++i, pinmask<<=1)
  {
    if (raising[i] != 0 && falling[i] > raising[i])
    {
      // Found raising and after that a falling edge
      unsigned int distance = falling[i] - raising[i];
      if (distance > (16 * 255))
        m_distance[i] = 255;
      else
        m_distance[i] = distance / 16;
      m_errors[i] = 0;
    }
    else if (raising[i] != 0 && (port & pinmask) != 0)
    {
      // Found raising edge and pin is still high at overflow
      m_distance[i] = 255;
      m_errors[i] = 0;
    }
    else
    {
      // probably signal error
      if (++m_errors[i] == 3)
      {
        m_distance[i] = 0; // set to error...
      }
    }
  }
  
  // Measurement reset
  m_pinchange_count = 0;
  m_distance_updated = true;
}

