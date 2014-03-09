/*
 * HMC5883L 3-axis compas module.
 *
 * Issue: compensation is required to correct for interference from the car's electronics and wiring. These
 *        cause reading issues. To correct, a calibration routine is required (since the wiring is bound to
 *        slightly modify every time). We need to store this data somewhere.
 *        See http://www.jameco.com/Jameco/Products/ProdDS/2150248.pdf for extensive documentation of
 *        performing corrections.
 *
 */

#include <Wire.h>

static int heading;
static char calibration_y;
static char calibration_z;
static unsigned char compas_data[6];
static int data_x() { return int(  ( ((unsigned int)compas_data[0]) << 8) | compas_data[1]  ); }
static int data_y() { return int(  ( ((unsigned int)compas_data[2]) << 8) | compas_data[3]  ); }
static int data_z() { return int(  ( ((unsigned int)compas_data[4]) << 8) | compas_data[5]  ); }

#define HMC5883L_ADDRESS ((unsigned char)0x1E)
#define REG_CONFIGA 0x00
#define REG_CONFIGB 0x01
#define REG_MODE 0x02
#define REG_DATA_BEGIN 0x03

#define MEASUREMENT_CONTINUOUS 0x00
#define MEASUREMENT_SINGLESHOT 0x01
#define MEASUREMENT_IDLE 0x03

static char compas_fast_arctan(unsigned int x, unsigned int y);

static void compas_i2c_read(unsigned char reg, unsigned char *data, unsigned char length)
{
  unsigned char ret;
  
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false); // keep bus occupied...
  
  Wire.requestFrom(HMC5883L_ADDRESS, length);
  Wire.endTransmission();

  while(Wire.available())
  {
    *(data++) = Wire.read();
    if (--length == 0)
      return;
  }
}

static void compas_i2c_write(unsigned char reg, unsigned char data)
{
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void compas_setup()
{
  /*
   * Configure compas to provide continuous measurements and give frequent results.
   * TODO: Decide whether to use calibration routine
   */
  
  heading = 0;
  compas_i2c_write(REG_CONFIGA, 0x58); // average=4; rate=75Hz; mode=normal
  compas_i2c_write(REG_CONFIGB, 0x20); // default gain, alter when having overflows
  compas_i2c_write(REG_MODE, MEASUREMENT_CONTINUOUS);
}

void compas_read()
{
  // read data
  compas_i2c_read(REG_DATA_BEGIN, compas_data, 6);
  // compute angle: [north, east, south, west] = [0, 64, -128, -64]
  heading = compas_fast_arctan(data_z(), data_x());
}

int compas_get_heading()
{
  return heading;
}

int compas_get_x()
{
  return data_x();
}

int compas_get_y()
{
  return data_y();
}

int compas_get_z()
{
  return data_z();
}

static unsigned int compas_fast_arctan(int x, int y)
{
  // Inspired by: http://www.romanblack.com/integer_degree.htm
  
   // Fast XY vector to integer degree algorithm - Jan 2011 www.RomanBlack.com
   // Converts any XY values including 0 to a degree value that should be
   // within +/- 1 degree of the accurate value without needing
   // large slow trig functions like ArcTan() or ArcCos().
   // NOTE! at least one of the X or Y values must be non-zero!
   // This is the full version, for all 4 quadrants and will generate
   // the angle in integer degrees from 0-360.
   
   // Any values of X and Y are usable including negative values provided
   // they are between -1456 and 1456 so the 16bit multiply does not overflow.
   while(x > 1456 || x < -1456 || y > 1456 || y < -1456)
   {
     x >>= 1;
     y >>= 1;
   }

   unsigned char tempdegree;
   unsigned char comp;
   unsigned int degree;     // this will hold the result

   // Save the sign flags then remove signs and get XY as unsigned ints
   unsigned char negflag = 0;
   if(x < 0)
   {
      negflag += 0x01;    // x flag bit
      x = (0 - x);        // is now +
   }
   unsigned int ux = x;                // copy to unsigned var before multiply
   if(y < 0)
   {
      negflag += 0x02;    // y flag bit
      y = (0 - y);        // is now +
   }
   unsigned int uy = y;                // copy to unsigned var before multiply

   // 1. Calc the scaled "degrees"
   if(ux > uy)
   {
      degree = (uy * 45) / ux;   // degree result will be 0-45 range
      negflag += 0x10;    // octant flag bit
   }
   else
   {
      degree = (ux * 45) / uy;   // degree result will be 0-45 range
   }

   // 2. Compensate for the 4 degree error curve
   comp = 0;
   tempdegree = degree;    // use an unsigned char for speed!
   if(tempdegree > 22)      // if top half of range
   {
      if(tempdegree <= 44) comp++;
      if(tempdegree <= 41) comp++;
      if(tempdegree <= 37) comp++;
      if(tempdegree <= 32) comp++;  // max is 4 degrees compensated
   }
   else    // else is lower half of range
   {
      if(tempdegree >= 2) comp++;
      if(tempdegree >= 6) comp++;
      if(tempdegree >= 10) comp++;
      if(tempdegree >= 15) comp++;  // max is 4 degrees compensated
   }
   degree += comp;   // degree is now accurate to +/- 1 degree!

   // Invert degree if it was X>Y octant, makes 0-45 into 90-45
   if(negflag & 0x10) degree = (90 - degree);

   // 3. Degree is now 0-90 range for this quadrant,
   // need to invert it for whichever quadrant it was in
   if(negflag & 0x02)   // if -Y
   {
      if(negflag & 0x01)   // if -Y -X
            degree = (180 + degree);
      else        // else is -Y +X
            degree = (180 - degree);
   }
   else    // else is +Y
   {
      if(negflag & 0x01)   // if +Y -X
            degree = (360 - degree);
   }
   return degree;
}
