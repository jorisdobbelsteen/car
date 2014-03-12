#include <RF24.h>
#include <Serial.h>

// relatively random number...
const uint64_t pipe_rf_car = 0xD783D63301LL; // rf to car communication
const uint64_t pipe_car_rf = 0xD783D63302LL; // car to rf communication

RF24 radio(49, 53);

void radio_setup()
{
  pinMode(49, OUTPUT);
  pinMode(53, OUTPUT);
  
  radio.begin();
  radio.setChannel(126);
  
  radio.openWritingPipe(pipe_car_rf);
//  radio.openReadingPipe(1, pipe_rf_car);

  radio.printDetails();
}

static unsigned int last_broadcast = 0;
static unsigned char sendbuffer[32];

void radio_tick()
{
  if (((unsigned int)millis() - last_broadcast) > 1000)
  {
    // broadcast every 1 second after previous, we got max 32 bytes of data
    sendbuffer[ 0] = 1; // message 1, general status
    sendbuffer[ 1] = distance_forward(); // distances in cm
    sendbuffer[ 2] = distance_forward_left();
    sendbuffer[ 3] = distance_forward_right();
    sendbuffer[ 4] = distance_rear_left();
    sendbuffer[ 5] = distance_rear_right();
    sendbuffer[ 6] = compas_get_heading() >> 8;
    sendbuffer[ 7] = compas_get_heading() & 0xFF;
    sendbuffer[ 8] = compas_get_x() >> 8;
    sendbuffer[ 9] = compas_get_x() & 0xFF;
    sendbuffer[10] = compas_get_y() >> 8;
    sendbuffer[11] = compas_get_y() & 0xFF;
    sendbuffer[12] = compas_get_z() >> 8;
    sendbuffer[13] = compas_get_z() & 0xFF;
    
    // send data...
    // FIXME: this can block for 500 ms, which is unacceptable!!!
    //        currently it blocks noticable!!!
    bool res = radio.write(sendbuffer, 14);
    if (!res)
      Serial.println("Radio tx failed");
    
    last_broadcast = millis();    
  }
}

