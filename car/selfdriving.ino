// Constants
const unsigned char headlight_dipped = 50;
const unsigned char headlight_main = 200;
const unsigned char rearlight_normal = 40;

const char power_forward_normal = 63;
const char power_forward_accelerate = 90;
const unsigned char power_forward_accelerate_duration = 100; //ms
const char power_reverse_normal = -62;
const char power_reverse_accelerate = -70;
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
  STATE_ResolveStart,
  STATE_ResolveDecision,
  STATE_ResolveForward,
  STATE_ResolveReverse,
  STATE_SoftOff,
  STATE_SoftOffWait,
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

/*
 * Variables for various states
 */
// ReverseDecision
unsigned char m_ReverseDecision_count;
unsigned char m_ReverseDecision_left[5];
unsigned char m_ReverseDecision_right[5];
 // Reverse state
int           m_Reverse_CompassStart;
// Resolve*** states
bool          m_Resolve_direction;
unsigned char m_Resolve_count;
unsigned char m_Resolve_forward[5];
unsigned char m_Resolve_forward_left[5];
unsigned char m_Resolve_forward_right[5];
unsigned char m_Resolve_rear_left[5];
unsigned char m_Resolve_rear_right[5];
// SoftOff
unsigned char m_SoftOff_count;

/*
 * State machine
 */
static unsigned char battery_delay;
void selfdriving_loop()
{
  // override for low battery detection...
  if(++battery_delay == 0 && batt_get_millivolt() < 6800 && current_state != STATE_BatteryLow) /* If voltage is below 6.9 volt, shutdown, if not already done */ { current_state = STATE_BatteryLow; entering = true; }
  else if(current_state != STATE_BatteryLow && current_state != STATE_SoftOff && !utils_program_enable()) /* If program switch does not authorize driving */ { current_state = STATE_SoftOff; entering = true; }
  
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
      StartWaiting();
    }
    bool atSpeed = WaitingFor(500);
    if(   (atSpeed &&  (distance_forward() < 70
                        || distance_forward_left() < 30
                        || distance_forward_right() < 30))
       || (!atSpeed && (distance_forward() < 60
                        || distance_forward_left() < 15
                        || distance_forward_right() < 15))
      )
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
      drivetrain_set_power(-80); // extensive breaking
      lights_set_breaklight_on();
      lights_set_rearlight(255); // break...
      StartWaiting();
    }
    if (WaitingFor(1000)) // for 500 ms
    {
      drivetrain_set_power(0);
      lights_set_rearlight(rearlight_normal);
      lights_set_breaklight_off();
      NEXT_STATE(ReverseDecision);
    }
    else if (WaitingFor(400)) // after initial extensive break, just normally
    {
      drivetrain_set_power(-40); // normal breaking, prevent reversing
    }
  }
  ATSTATE(ReverseDecision)
  {
    bool left = false;
    bool right = false;
    if(entering)
    {
      m_ReverseDecision_count = 0; // reset measurement counter
      left = drivetrain_get_steer() < -10; // was going left...
      right = drivetrain_get_steer() > 10; // was going right
    }
    else
    {
      if (distance_updated()) // take measurement counter
      {
        m_ReverseDecision_left[m_ReverseDecision_count] = distance_forward_left();
        m_ReverseDecision_right[m_ReverseDecision_count] = distance_forward_right();
        m_ReverseDecision_count++;
        
        if (m_ReverseDecision_count == 5)
        {
          // pick median
          utils_sort(m_ReverseDecision_left, 5);
          utils_sort(m_ReverseDecision_right, 5);
          unsigned char left_distance = m_ReverseDecision_left[2];
          unsigned char right_distance = m_ReverseDecision_right[2];
          // make random decision... but more likely a informed decision --> 1 to 15
          unsigned char randval = millis();
          bool dorandom = (randval & 0xf0) == 0;
          // actual decision
          if((dorandom && (randval & 0x1)) || (!dorandom && left_distance > right_distance))
            left = true;
          else
            right = true;
            
          lights_set_headlight(headlight_dipped);
        }
        else
        {
          lights_set_headlight(headlight_main); // flash headlights a bit...
        }
      }
    }
    if(left) // was going left or more distance there...
    {
      // turn right when reversing, so front lines up to left
      lights_set_indicator_right();
      drivetrain_set_steer(110);
      NEXT_STATE(Reverse);
    }
    else if (right)  // steer postive (forward), so negative (reverse)
    {
      // turn left when reversing, so front lines up to right
      lights_set_indicator_left();
      drivetrain_set_steer(-110);
      NEXT_STATE(Reverse);
    }
  }
  ATSTATE(Reverse)
  {
    if(entering)
    {
      compas_read();
      m_Reverse_CompassStart = compas_get_heading();
      lights_set_headlight(1);
      drivetrain_set_power(power_reverse_accelerate);
      StartWaiting();
    }
    // Get heading compute delta
    compas_read();
    int deltaHeading = abs(compas_get_heading() - m_Reverse_CompassStart);
    while(deltaHeading >= 360) deltaHeading -= 360; // remove full rotation
    // decision...
    if(WaitingFor(2000)
      || (WaitingFor(600) && deltaHeading > 50) // max 50 degree turn (very approximately...) + some overshoot from stopping... Give at least 0.6 seconds always!
      || distance_rear_left() < 38 // only 38 cm margin, since we should be going slowly backward, so would work?
      || distance_rear_right() < 38)
    {
      lights_set_indicator_off();
      drivetrain_set_steer(0); // choose left or right at random
      drivetrain_set_power(0);
      if(distance_forward() < 75) // getting closeby, so driving forward will abort anyways...
        NEXT_STATE(ResolveStart);
      else
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
      Serial.print(" turn ");
      Serial.print(deltaHeading);
      Serial.print(" deg batt ");
      Serial.print(batt_get_millivolt());
      Serial.print(" mv");
      Serial.println();
    }
  }
  /*
   * Resolving is a more powerful turning logic that scans to go a certain direction
   */
  ATSTATE(ResolveStart)
  {
    // decide direction...
    m_Resolve_direction = millis() & 0x1;
    lights_set_indicator_both();
    NEXT_STATE(ResolveDecision);
  }
  ATSTATE(ResolveDecision)
  {
    // measure a bit ...
    if(entering)
    {
      m_Resolve_count = 0; // reset measurement counter
      distance_updated();
    }
    else if (distance_updated()) // take measurement counter
    {
      m_Resolve_forward[m_Resolve_count] = distance_forward();
      m_Resolve_forward_left[m_Resolve_count] = distance_forward_left();
      m_Resolve_forward_right[m_Resolve_count] = distance_forward_right();
      m_Resolve_rear_left[m_Resolve_count] = distance_rear_left();
      m_Resolve_rear_right[m_Resolve_count] = distance_rear_right();
      m_Resolve_count++;
      
      if (m_Resolve_count == 5)
      {
        utils_sort(m_Resolve_forward, 5);
        utils_sort(m_Resolve_forward_left, 5);
        utils_sort(m_Resolve_forward_right, 5);
        utils_sort(m_Resolve_rear_left, 5);
        utils_sort(m_Resolve_rear_right, 5);
        
        if (m_Resolve_forward[2] > 180
            && m_Resolve_forward_left[2] > 35
            && m_Resolve_forward_right[2] > 35)
        {
          // Plenty of space, so we can go ...
          drivetrain_set_steer(0);
          lights_set_indicator_off();
          NEXT_STATE(DriveForward);
        }
        else
        {
          unsigned char forward_side = min(m_Resolve_forward_left[2], m_Resolve_forward_right[2]);
          unsigned char forward = min(m_Resolve_forward[2], (forward_side > 127) ? 255 : (forward_side * 2)); // left and right sensors get a little benefit...
          unsigned char reverse = min(m_Resolve_rear_left[2], m_Resolve_rear_right[2]);
          if (forward > reverse) // more distance forward
            NEXT_STATE(ResolveForward);
          else
            NEXT_STATE(ResolveReverse);
        }
      }
    }
  }
  ATSTATE(ResolveForward)
  {
    if(entering)
    {
      StartWaiting();
      drivetrain_set_steer(m_Resolve_direction ? -110 : 110);
      drivetrain_set_power(power_forward_accelerate);
    }
    if (WaitingFor(650))
    {
      NEXT_STATE(ResolveDecision);
    }
//    else if (WaitingFor(600))
//    {
//      drivetrain_set_power(-10);
//    }
    else if(WaitingFor(power_forward_accelerate_duration))
    {
      drivetrain_set_power(power_forward_normal);
    }
    if(exiting)
    {
      drivetrain_set_power(0);
    }
  }
  ATSTATE(ResolveReverse)
  {
    if(entering)
    {
      StartWaiting();
      drivetrain_set_steer(m_Resolve_direction ? 110 : -110);
      drivetrain_set_power(power_reverse_accelerate);
    }
    if (WaitingFor(650))
    {
      NEXT_STATE(ResolveDecision);
    }
//    else if (WaitingFor(600))
//    {
//      drivetrain_set_power(10);
//    }
    else if(WaitingFor(power_reverse_accelerate_duration))
    {
      drivetrain_set_power(power_reverse_normal);
    }
    if(exiting)
    {
      drivetrain_set_power(0);
    }
  }
  ATSTATE(SoftOff)
  {
    if(entering)
    {
      // TODO: SHOULD BREAK!!!
      lights_set_headlight(0);
      lights_set_rearlight(0);
      lights_set_breaklight_off();
      lights_set_indicator_both();
      drivetrain_set_power(0);
      drivetrain_set_steer(0);
      
      StartWaiting();
      m_SoftOff_count = 0; // used for blinking rear lights
    }
    if(utils_program_enable())
    {
      NEXT_STATE(SoftOffWait);
    }
    else if(WaitingFor(1000))
    {
      m_SoftOff_count++;
      if (m_SoftOff_count & 0x1)
      {
        lights_set_breaklight_on();
        lights_set_rearlight(255);
      }
      else
      {
        lights_set_breaklight_off();
        lights_set_rearlight(0);
      }
      StartWaiting();
    }
    if(CanPrintStatus())
    {
      Serial.print("Driving disabled. Heading ");
      compas_read();
      Serial.print((int)compas_get_heading());
      Serial.print(" deg; batt ");
      Serial.print(batt_get_millivolt());
      Serial.println(" mv");
    }
  }
  ATSTATE(SoftOffWait) // give couple seconds before actually starting
  {
    if(entering)
    {
      // TODO: SHOULD BREAK!!!
      lights_set_headlight(headlight_dipped);
      lights_set_rearlight(255);
      lights_set_breaklight_on();
      lights_set_indicator_both();
      
      StartWaiting();
    }
    if(WaitingFor(3000))
    {
      lights_set_headlight(headlight_dipped);
      NEXT_STATE(Start);
    }
    else if(WaitingFor(2000))
    {
      lights_set_indicator_off();
      lights_set_headlight(headlight_main);
    }
    if(CanPrintStatus())
    {
      Serial.println("Driving engaging soon...");
    }
  }
  ATSTATE(BatteryLow)
  {
    // Turn off most systems -- dead end...
    if(entering)
    {
      // TODO: SHOULD BREAK!!!
      lights_set_headlight(0);
      lights_set_rearlight(0);
      lights_set_breaklight_off();
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

