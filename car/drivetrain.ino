/*
 * Engine and steering servo controller
 *
 * API
 *   void drivetrain_setup()
 *       Initializes the controllers to the servo's.
 *   signed char drivetrain_get_power()
 *   void drivetrain_set_power(signed char power)
 *       Sets power to the entire.
 *       0 is stopped
 *       Positive values mean move forward, where 127 is 100% power. Recommended limit is 80.
 *       Negative values mean move backward, where -127 is 100% power. Recommended limit is -80.
 *   
 */

#define SERVO_MOTOR_DRIVE_CENTER 374  // zero position
#define SERVO_MOTOR_DRIVE_OFFSET   5  // offset for sufficient engine power
// NOTE (abs(SERVO_MOTOR_DRIVE_SPEED) + abs(SERVO_MOTOR_DRIVE_BATTERY_COMPENSATION)) < 60 !!! OTHERWISE MODIFY CODE
#define SERVO_MOTOR_DRIVE_SPEED   38  // fraction of "speed" applied to this offset.
#define SERVO_MOTOR_DRIVE_BATTERY_COMPENSATION   -8 // PWM value compensation per volt above batery center voltage.  // 1 volt @ 7.4 volt means 30% power
#define SERVO_MOTOR_DRIVE_BATTERY_CENTER       7400 // battery center value
#define SERVO_MOTOR_SPEED_CUTOFF   2  // -SERVO_MOTOR_SPEED_CUTOFF till SERVO_MOTOR_SPEED_CUTOFF is treated as engine OFF


#define SERVO_STEERING_CENTER 373
#define SERVO_STEERING_SWING 72

static signed char m_power_compenstation;
static signed char m_power;
static signed char m_steer;

void drivetrain_setup()
{
  // compute compenstation
  drivetrain_compute_compenstation();
  
  // Timer 3
  // WGM3:0 = mode 14 (count up to ICR3)
  // CS[2:0] = 3, prescaler div 64
  //
  //       Clock  / prescaler / counter TOP = PWM period
  //       16 MHz / 64       / 4096 (ICR3) = 61 Hz
  //          Note: servo's require frequency between 50 and 150 Hz (wikipedia)
  //
  //    output A => digital pin 5
  //    output B => digital pin 2
  //    output C => digital pin 3
  pinMode(5, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  ICR3 = 4096;
  TCCR3A = _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1) | _BV(WGM31); // COMnx[1:0] = clear on compare, WGM[1:0] see up
  TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS31) | _BV(CS30); // WGM[3:2] and CS[2:0] see up
  TCCR3C = 0; // FOCnx = 0
  // set reserved pin to 1.5 ms pulse
  OCR3A = 380; // 1.5ms
  // reset controlled pins
  drivetrain_set_power(0);
  drivetrain_set_steer(0);
}

void drivetrain_compute_compenstation()
{
  {
    // compute compensation for battery voltage
    int diff_mv = (batt_get_millivolt() - SERVO_MOTOR_DRIVE_BATTERY_CENTER); // difference varies around 1000 volt
    m_power_compenstation = (diff_mv * SERVO_MOTOR_DRIVE_BATTERY_COMPENSATION) >> 10; // divide by 1024: milliVolt to Volt, and multiply by two for drivetrain_set_power
  }
}

static unsigned int m_drivetrain_lastcompenstation;
void drivetrain_tick()
{
  if (((unsigned int)millis() - m_drivetrain_lastcompenstation) > 5000)
  {
    drivetrain_compute_compenstation();
    m_drivetrain_lastcompenstation = millis();
  }
}

signed char drivetrain_get_power()
{
  return m_power;
}

void drivetrain_set_power(signed char power) // [-127,127]: 0 is stopped, positive = forward, negative = backward
{
  m_power = power;
  int pwmValue = (((SERVO_MOTOR_DRIVE_SPEED*2 + m_power_compenstation) * (long)power) >> 8);
  if(power < -SERVO_MOTOR_SPEED_CUTOFF) // backward
    OCR3C = SERVO_MOTOR_DRIVE_CENTER - SERVO_MOTOR_DRIVE_OFFSET + pwmValue;
  else if (power > SERVO_MOTOR_SPEED_CUTOFF) // forward
    OCR3C = SERVO_MOTOR_DRIVE_CENTER + SERVO_MOTOR_DRIVE_OFFSET + pwmValue;
  else // stopped
    OCR3C = SERVO_MOTOR_DRIVE_CENTER;
}

signed char drivetrain_get_steer()
{
  return m_steer;
}

void drivetrain_set_steer(signed char direction) // [-127,127]: -127 is left, 0 is center, 127 is right
{
  m_steer = direction;
  OCR3B = SERVO_STEERING_CENTER + (((SERVO_STEERING_SWING*2) * (long)direction) >> 8);
}

