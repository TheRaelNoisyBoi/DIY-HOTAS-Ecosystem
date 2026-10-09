/***************************************
  I2C Scanner - Noisy Boi HOTAS ecosystem
  - Lists every device that answers on the I2C bus (SDA = pin 2, SCL = pin 3).
  - Serial Monitor: 115200 baud (same as all device firmwares).
  - Expected addresses:
      HFB Joystick (grip attached): 0x20, 0x21, 0x48, 0x49, 0x50, 0x5A
      Twin Engine Throttle        : 0x20, 0x21, 0x4A
      Black Shark Collective      : 0x20, 0x21 (grip attached)
      Rudder Pedals               : no I2C devices
***************************************/

#include <Wire.h>

#define SERIAL_BAUD 115200

void setup()
{
  Wire.begin();

  Serial.begin(SERIAL_BAUD);
  while (!Serial);             // Leonardo / Pro Micro: wait for the serial monitor
  Serial.println("\nI2C Scanner");
}


void loop()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // A device acknowledges its address when endTransmission() returns 0
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}
