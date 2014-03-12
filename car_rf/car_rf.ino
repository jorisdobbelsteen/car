// Arduino Uno

#include <Serial.h>
#include <SPI.h>
#include <RF24.h>

// relatively random number...
const uint64_t pipe_rf_car = 0xD783D63301LL; // rf to car communication
const uint64_t pipe_car_rf = 0xD783D63302LL; // car to rf communication

RF24 radio(9, 10);

void setup()
{
  Serial.begin(57600);
  printf_begin();
  Serial.println("Car-RF. Car remote communication unit");
  
  SPI.begin();
  
  radio.begin();
  radio.setChannel(126);
  
//  radio.openWritingPipe(pipe_rf_car);
  radio.openReadingPipe(1, pipe_car_rf);
  
  radio.startListening();
  
  radio.printDetails();
}

static unsigned char recvbuffer[32];

static int getint(unsigned char* data)
{
  return (int)(  (((unsigned int)data[1]) << 8) | (data[1])  );
}

void loop()
{
  if (radio.available())
  {
    Serial.println("Radio data available...");
    while(radio.read(recvbuffer, 32))
    {
      if (recvbuffer[0] == 1)
      {
        Serial.print("Status: ");
        Serial.println("not parsed");
//      sendbuffer[ 1] = distance_forward(); // distances in cm
//      sendbuffer[ 2] = distance_forward_left();
//      sendbuffer[ 3] = distance_forward_right();
//      sendbuffer[ 4] = distance_rear_left();
//      sendbuffer[ 5] = distance_rear_right();
//      sendbuffer[ 6] = compas_get_heading() >> 8;
//      sendbuffer[ 7] = compas_get_heading() & 0xFF;
//      sendbuffer[ 8] = compas_get_x() >> 8;
//      sendbuffer[ 9] = compas_get_x() & 0xFF;
//      sendbuffer[10] = compas_get_y() >> 8;
//      sendbuffer[11] = compas_get_y() & 0xFF;
//      sendbuffer[12] = compas_get_z() >> 8;
//      sendbuffer[13] = compas_get_z() & 0xFF;
      }
      else
      {
        Serial.print("Unknown message type #");
        Serial.println((unsigned int)recvbuffer[0]);
      }
    }
  }
}

// RF24 debug support
int serial_putc( char c, FILE * ) 
{
  Serial.write( c );
  return c;
} 
void printf_begin(void)
{
  fdevopen( &serial_putc, 0 );
}

