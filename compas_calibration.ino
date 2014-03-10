static int calib_x_min, calib_y_min, calib_z_min;
static int calib_x_max, calib_y_max, calib_z_max;

#include <EEPROM.h>

#define EEPROM_COMPAS_ADDR 1024

void compass_calibration_load_from_eeprom()
{
  compas_set_calibration(EEPROM_read_int(EEPROM_COMPAS_ADDR), EEPROM_read_int(EEPROM_COMPAS_ADDR+2), EEPROM_read_int(EEPROM_COMPAS_ADDR+4));
}

static void compas_calibration_print()
{
  Serial.print("  Calibration [");
  Serial.print(compas_get_calibration_offset_x());
  Serial.print("; ");
  Serial.print(compas_get_calibration_offset_y());
  Serial.print("; ");
  Serial.print(compas_get_calibration_offset_z());
  Serial.println("]");
}

void compas_calibration_setup()
{
  Serial.println("Press 'p' to print current values, 'a' to adjust heading, 'w' to adjust and write to EEPROM, 'r' to reset to zero");
  compas_calibration_print();
  
  compas_read();
  calib_x_min = calib_x_max = compas_get_x();
  calib_y_min = calib_y_max = compas_get_y();
  calib_z_min = calib_z_max = compas_get_z();
}

static unsigned char period = 0;

void compas_calibration_loop()
{
  delay(1000/75);
  
  compas_read();

  int x = compas_get_x();
  int y = compas_get_y();
  int z = compas_get_z();
  
  if (x < calib_x_min)
    calib_x_min = x;
  if (y < calib_y_min)
    calib_y_min = y;
  if (z < calib_z_min)
    calib_z_min = z;

  if (x > calib_x_max)
    calib_x_max = x;
  if (y > calib_y_max)
    calib_y_max = y;
  if (z > calib_z_max)
    calib_z_max = z;

  bool printval = false;
  bool adjust = false;
  bool store = false;
  bool reset = false;
  while(Serial.available())
  {
    char c = Serial.read();
    printval |= (c == 'p');
    adjust |= (c == 'a');
    store |= (c == 'w');
    reset |= (c == 'r');
  }
  
  if (printval)
  {
    compas_calibration_print();
  }
  else if (reset)
  {
    compas_set_calibration(0, 0, 0);
    EEPROM_write_int(EEPROM_COMPAS_ADDR, 0);
    EEPROM_write_int(EEPROM_COMPAS_ADDR+2, 0);
    EEPROM_write_int(EEPROM_COMPAS_ADDR+4, 0);
    Serial.println("  Calibration reset [0; 0; 0] stored!");
    calib_x_min = calib_x_max = x;
    calib_y_min = calib_y_max = y;
    calib_z_min = calib_z_max = z;
  }
  else if (adjust || store)
  {
    int x_mid = (calib_x_min + calib_x_max) / 2;
    int y_mid = (calib_y_min + calib_y_max) / 2;
    int z_mid = (calib_z_min + calib_z_max) / 2; z_mid = 0; // For now, ignore the z axis...
    Serial.print("  Calibration [");
    Serial.print(x_mid);
    Serial.print("; ");
    Serial.print(y_mid);
    Serial.print("; ");
    Serial.print(z_mid);
    Serial.print("]");
    compas_set_calibration(x_mid, y_mid, z_mid);
    if (store)
    {
      EEPROM_write_int(EEPROM_COMPAS_ADDR, x_mid);
      EEPROM_write_int(EEPROM_COMPAS_ADDR+2, y_mid);
      EEPROM_write_int(EEPROM_COMPAS_ADDR+4, z_mid);
      Serial.print(" stored!");
    }
    Serial.println();
  }
  
  if ((period++ & 0xf) == 0 || adjust || store)
  {
    Serial.print("Compas: ");
    Serial.print((int)compas_get_heading());
    Serial.print(" [");
    Serial.print(compas_get_x());
    Serial.print("; ");
    Serial.print(compas_get_y());
    Serial.print("; ");
    Serial.print(compas_get_z());
    Serial.print("] limits [");
    Serial.print(calib_x_min);
    Serial.print("/");
    Serial.print(calib_x_max);
    Serial.print("; ");
    Serial.print(calib_y_min);
    Serial.print("/");
    Serial.print(calib_y_max);
    Serial.print("; ");
    Serial.print(calib_z_min);
    Serial.print("/");
    Serial.print(calib_z_max);
    Serial.print("; ");
    Serial.println("] ");
  }
}

