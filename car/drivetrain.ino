/*
 * Engine and steering servo controller
 */

#define SERVO_MOTOR_DRIVE_CENTER 374  // zero position
#define SERVO_MOTOR_DRIVE_OFFSET -15  // offset for sufficient engine power
#define SERVO_MOTOR_DRIVE_SPEED  -25  // fraction of "speed" applied to this offset
#define SERVO_MOTOR_SPEED_CUTOFF 2    // -SERVO_MOTOR_SPEED_CUTOFF till SERVO_MOTOR_SPEED_CUTOFF is treated as engine OFF
#define SERVO_STEERING_CENTER 379
#define SERVO_STEERING_SWING 72

static signed char m_power;
static signed char m_steer;

void drivetrain_setup()
{
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

signed char drivetrain_get_power()
{
  return m_power;
}

void drivetrain_set_power(signed char power) // [-127,127]: 0 is stopped, positive = forward, negative = backward
{
  m_power = power;
  int pwmValue = (((SERVO_MOTOR_DRIVE_SPEED*2) * (long)power) >> 8);
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

