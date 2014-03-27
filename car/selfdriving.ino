// Constants
const unsigned char headlight_dipped = 60;
const unsigned char headlight_main = 180;
const unsigned char rearlight_normal = 60;

const char power_forward_normal = 57;
const char power_forward_accelerate = 66;
const unsigned char power_forward_accelerate_duration = 80; //ms
const char power_reverse_normal = -65;
const char power_reverse_accelerate = -80;
const unsigned char power_reverse_accelerate_duration = 80; //ms

// State machine helpers
#define BEGIN_STATEMACHINE enum SelfDrivingState next_state = current_state; switch(current_state) { default: {
#define ATSTATE(x)         } break; case STATE_##x: if (entering) Serial.println(">>" #x); {
#define END_STATEMACHINE   } break; } { entering = (current_state != next_state); current_state = next_state; }
#define NEXT_STATE(x)      do{next_state = (STATE_##x);}while(0)
static bool entering;
#define exiting (current_state != next_state)

// State machine current state
static enum SelfDrivingState
{
  STATE_Start,
  STATE_DriveForward,
  STATE_DriveForward2,
  STATE_BreakForward,
  STATE_ReverseDecision,
  STATE_Reverse,
  STATE_BatteryLow
} current_state;

/*
 * Helper functions
 *
 * Waiting loops:
 *  void StartWaiting()
 *  bool WaitingFor(unsigned int duration_milliseconds)
 *      WaitingFor will return true once <duration_milliseconds> milliseconds have passed since the
 *      last call to StartWaiting.
 *
 *  bool CanPrintStatus()
 *      Should be checked before periodic Serial.print calls, to prevent excessive debug output. Returns true once only if the
 *      previous time it returned true is 350 ms (or other configured time) ago.
 */


static unsigned char battery_delay;
void selfdriving_loop()
{
  // override for low battery detection...
  if(++battery_delay == 0 && batt_get_millivolt() < 6900 && current_state != STATE_BatteryLow) /* If voltage is below 6.9 volt, shutdown, if not already done */ { current_state = STATE_BatteryLow; entering = true; }
  
  BEGIN_STATEMACHINE
  ATSTATE(Start)
  {
    if(entering)
    {
      lights_set_headlight(1);
      lights_set_rearlight(1);
      lights_set_indicator_both();
    }
    if(distance_forward() > 40
        && distance_forward_left() > 25
        && distance_forward_right() > 25)
    {
      Serial.print("NEXT!");
      NEXT_STATE(DriveForward);
    }
    else if(CanPrintStatus())
    {
      Serial.print("wait [");
      selfdriving_print_distance_forward();
      Serial.println();
    }
    if(exiting)
    {
      lights_set_indicator_off();
    }
  }
  ATSTATE(DriveForward)
  {
    if(entering)
    {
      // use lights to indicate driving mode
      lights_set_headlight(headlight_dipped);
      lights_set_rearlight(rearlight_normal);
      
      // slowly move forward
      drivetrain_set_power( power_forward_accelerate ); // acceleration
      drivetrain_set_steer( 0 );
      StartWaiting();
    }
    if(WaitingFor(power_forward_accelerate_duration))
      NEXT_STATE(DriveForward2);
  }
  ATSTATE(DriveForward2)
  {
    if(entering)
    {
      // slowly move forward
      drivetrain_set_power( power_forward_normal );
      drivetrain_set_steer( 0 );
    }
    if(distance_forward() < 70                            // getting closeby
        || distance_forward_left() < 30
        || distance_forward_right() < 30)
    {
      NEXT_STATE(BreakForward);
    }
    else if (distance_forward_left() < 60 && distance_forward_right() > 60) // left getting closeby, but right is not
    {
      lights_set_indicator_right();
      drivetrain_set_steer( 100 );
    }
    else if (distance_forward_right() < 60 && distance_forward_left() > 60) // right getting closeby, but left is not
    {
      lights_set_indicator_left();
      drivetrain_set_steer( -100 );
    }
    else if (distance_forward_left() > 60 && distance_forward_right() > 60) // 5 cm "hysteris", to prevent excessive twitching...
    {
      lights_set_indicator_off();
      drivetrain_set_steer( 0 );
    }
    if(CanPrintStatus())
    {
      Serial.print("drive ");
      selfdriving_print_distance_forward();
      Serial.print(" batt ");
      Serial.print(batt_get_millivolt());
      Serial.print(" mv");
      Serial.println();
    }
    if (exiting)
    {
      lights_set_indicator_off();
      drivetrain_set_steer( 0 );
    }
  }
  ATSTATE(BreakForward)
  {
    if(entering)
    {
      // set drivetrain in reverse direction than before...
      drivetrain_set_power(-60); // extensive breaking
      lights_set_breaklight_on();
      StartWaiting();
    }
    if (distance_forward() < 20 || distance_forward_left() < 15 || distance_forward_right() < 15) // danger ... too close
    {
      lights_set_rearlight(255);
      drivetrain_set_power(-60); // hard breaking/reversing
    }
    else if (WaitingFor(500)) // for 500 ms
    {
      drivetrain_set_power(0);
      lights_set_rearlight(rearlight_normal);
      lights_set_breaklight_off();
      NEXT_STATE(ReverseDecision);
    }
    else if (WaitingFor(120)) // after initial extensive break, just normally
    {
      lights_set_rearlight(rearlight_normal);
      drivetrain_set_power(-20); // normal breaking
    }
  }
  ATSTATE(ReverseDecision)
  {
    if(entering)
    {
      StartWaiting();
    }
    bool left = entering && (drivetrain_get_steer() < -10);
    bool right = entering && (drivetrain_get_steer() > 10);
    if(!left && !right)
    {
      // make random decision... But can also make informed decision...
      if(millis() & 0x1)
        left = true;
      else
        right = true;
    }
    if(left) // steer negative (forward), so positive (reverse)
    {
      drivetrain_set_steer(110);
      NEXT_STATE(Reverse);
    }
    else if (right)  // steer postive (forward), so negative (reverse)
    {
      drivetrain_set_steer(-110);
      NEXT_STATE(Reverse);
    }
  }
  ATSTATE(Reverse)
  {
    if(entering)
    {
      lights_set_headlight(1);
      drivetrain_set_power(power_reverse_accelerate);
      StartWaiting();
    }
    if(WaitingFor(5000)
      || distance_rear_left() < 40
      || distance_rear_right() < 40)
    {
      drivetrain_set_steer(0); // choose left or right at random
      drivetrain_set_power(0);
      NEXT_STATE(DriveForward);
    }
    else if(WaitingFor(power_reverse_accelerate_duration))
    {
      drivetrain_set_power(power_reverse_normal);
    }
    if(CanPrintStatus())
    {
      Serial.print("reversing ");
      selfdriving_print_distance_rear();
      Serial.print(" batt ");
      Serial.print(batt_get_millivolt());
      Serial.print(" mv");
      Serial.println();
    }
  }
  ATSTATE(BatteryLow)
  {
    // Turn off most systems
    if(entering)
    {
      // TODO: SHOULD BREAK!!!
      lights_set_headlight(0);
      lights_set_rearlight(0);
      lights_set_indicator_both();
      drivetrain_set_power(0);
    }
    if(CanPrintStatus())
    {
      Serial.print("Battery low at ");
      Serial.print(batt_get_millivolt());
      Serial.println(" mv");
    }
  }
  END_STATEMACHINE
}

// Helper for limit amount of debug printing
static long status_timestamp;
static bool CanPrintStatus()
{
  long curtime = millis();
  if ((curtime - status_timestamp) >= 350) // print every that many ms
  {
    status_timestamp = curtime;
    return true;
  }
  else
  {
    return false;
  }
}

// Helper for delay waiting
static long timestamp;
static void StartWaiting()
{
  timestamp = millis();
}
static bool WaitingFor(unsigned int duration_milliseconds)
{
  return (millis() - timestamp) >= duration_milliseconds;
}

// Debug serial printing helpers
void selfdriving_print_distance_forward()
{
  Serial.print("[");
  Serial.print((unsigned int)distance_forward_left());
  Serial.print("; ");
  Serial.print((unsigned int)distance_forward());
  Serial.print("; ");
  Serial.print((unsigned int)distance_forward_right());
  Serial.print(" cm]");
}
void selfdriving_print_distance_rear()
{
  Serial.print("[");
  Serial.print((unsigned int)distance_rear_left());
  Serial.print("; ");
  Serial.print((unsigned int)distance_rear_right());
  Serial.print(" cm]");
}

// Startup code
void selfdriving_setup()
{ 
  // startup state machine
  current_state = STATE_Start;
  entering = true;
}

