/* GY-91 (MPU9250+BMP280) Basic Example Code
 by: Kris Winer, Steve Nelson
 date: Sept 21, 2016
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy Kris and Steve a beer some time.

 Demonstrate initializing and calibrating the GY-91 gyroscope, accelerometer, magnetometer, pressure and temperature sensor
 Display the raw values in the serial monitor.

 This is intended to work on the esp8266 12e nodemcu board through I2C, but it should work in just about any Arduino environment. The
 only difference are the pins the I2C connects to.

  Hardware setup:
 GY-91 Breakout --------- ESP8266 12e
 VIN ---------------------- 3.3V
 3v3 ---------------------- Not connected
 GND ---------------------- GND
 SCL ---------------------- D1 (GPIO5)
 SDA ---------------------- D2 (GPIO4)
 SOC/SAO ------------------ not connected
 NCS ---------------------- 3.3V
 CSB ---------------------- not connected

*/
#include "GY91.h"
#include "Adafruit_BMP280.h"

GY91 myGY91;
Adafruit_BMP280 myBMP280;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  I2Cscan();
  Serial.println("------------------------------------------------------");
  Serial.println("GY91 basic example");
  Serial.println("by Steve Nelson adapted from Kris Winer's MPU9250 codebase");
  Serial.println("------------------------------------------------------");
  myGY91.begin(false);
  myBMP280.begin(BMP280_ADDRESS);
  delay(1000);
}

float temperature=0;

void loop() {
  //This updates the data values 
  myGY91.update();
  displayData();
  delay(500);
}

void displayData(){
  Serial.println("GY91 Raw Data");
  Serial.println("-------------");
  Serial.println("Yaw, Pitch, Roll: ");
  Serial.print(myGY91.yaw, 1);
  Serial.print(", ");
  Serial.print(myGY91.pitch,1);
  Serial.print(", ");
  Serial.println(myGY91.roll,1);
  Serial.println("temperature, pressure:");
  Serial.print(myBMP280.readTemperature(), 1);
  Serial.print("C, ");
  Serial.print(myBMP280.readPressure(),1);
  Serial.println("bar ");
  Serial.print("altitude:");
  Serial.print(myBMP280.readAltitude(1017.23), 1);
  Serial.println(" meters");
  
}

void I2Cscan()
{
// scan for i2c devices
  byte error, address;
  int nDevices;
  
  
  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.println("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0){
    Serial.println("No I2C devices found\n");
  }
  else{
    Serial.println("done\n");
  }
}
