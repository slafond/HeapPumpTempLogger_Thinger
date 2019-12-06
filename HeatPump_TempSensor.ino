// Basic temperature logger on a MKR1010 for 18B20 temperature sensors
// Logging all data on a thinger instance
//


#include <WiFiNINA.h>
#include <ThingerWiFiNINA.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define USERNAME "thinger_username"                    // your thinger username
#define DEVICE_ID "thinger_device_id"     	       // your thinger device id
#define DEVICE_CREDENTIAL "thinger_deive_credential"   // your thinger device credential

#define SSID "ssid_name"             //SSID of your WiFi
#define SSID_PASSWORD "ssid_pwd"     //Password of your WiFi

//#define SearchSensorAdd //Uncomment if you want to display on the serial port the addresses of all connected 18B20 sensors
//#define DEBUG //Uncoment to pritou debugging info

OneWire ds(1);  //data wire connected on pin ds (4.7K resistor necessary)
DallasTemperature sensors(&ds); //https://github.com/milesburton/Arduino-Temperature-Control-Library

const int NbSensor = 9; //Number of 18B20 temperature sensors

ThingerWiFiNINA thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);

DeviceAddress sensorAdd[NbSensor] = {  // to be updated with the actual sensor addresses
{ 0x28, 0xFF, 0x64, 0x1D, 0x8F, 0xC0, 0x8A, 0x98 }, //address of sensor 1 - Ground Out
{ 0x28, 0xFF, 0x64, 0x1D, 0x8F, 0xD3, 0xDF, 0xC5 }, //address of sensor 2 - Ground In
{ 0x28, 0xFF, 0x64, 0x1D, 0x8F, 0xD3, 0xF6, 0x7A }, //address of sensor 3 - Pump Out
{ 0x28, 0xFF, 0x64, 0x1D, 0x8F, 0xD4, 0xB6, 0x52 }, //address of sensor 4 - Pump In
{ 0x28, 0xFF, 0x64, 0x1D, 0x8F, 0xAC, 0xA2, 0x6E },  //address of sensor 5 - Buffer Out
{ 0x28, 0xFF, 0x64, 0x1D, 0x8F, 0xC3, 0x61, 0x4 },  //address of sensor 6 - Buffer In
{ 0x28, 0xD0, 0x6, 0x79, 0x97, 0x2, 0x3, 0x5C },   //address of sensor 7 - Chaufferie
{ 0x28, 0x4A, 0x20, 0x79, 0x97, 0x2, 0x3, 0x46 },  //address of sensor 8 - Garage in
{ 0xFF, 0x20, 0x35, 0x35, 0x91, 0x2, 0x2, 0x0 }      //address of sensor 9 - Outddor temp.
};   

// global variable holding the latest measurments
float tempMeasurments[NbSensor];

void setup() {
  // configure wifi network
  thing.add_wifi(SSID, SSID_PASSWORD);

  // start the serial port
  Serial.begin(9600);

  // Start up the DallasTemperature library
  sensors.begin();

  // Definition of the variables to be logged on thinger.io 

      thing["temperatures"] >> [](pson& out)
      {
      out["GroundOut"] = tempMeasurments[1];
      out["GroundIn"] = tempMeasurments[0];
      out["PumpOut"] = tempMeasurments[2];
      out["PumpIn"] = tempMeasurments[3];
      out["BufferOut"] = tempMeasurments[5];
      out["BufferIn"] = tempMeasurments[4];
      out["Chaufferie"] = tempMeasurments[6];
      out["Garage"] = tempMeasurments[7];
      out["Out"] = tempMeasurments[8];
      };
 
  String mess = "Number of sensor addresses registered:";
  mess = mess + NbSensor;
  mess = mess + "\n";
  Serial.print(mess);

  // locate temp. sensors on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

   // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  // set sensore resolution 9 to 12 bits
  for( int i=0; i<0; i++) {
  sensors.setResolution(sensorAdd[i], 12);
  }

  // more details at http://docs.thinger.io/arduino/
}

void loop() {
  thing.handle();

 #ifdef DEBUG
   Serial.print("Requesting temperatures...");
 #endif  
 
   sensors.requestTemperatures(); // Send the command to get temperatures

 #ifdef DEBUG
   Serial.println("DONE");
 #endif

 returnAllTemperature(tempMeasurments); // fetch all temperature measurments into the array tempMeasurments


//****************************
// Check for sensor addresses
//***************************
#ifdef SearchSensorAdd
byte addr[8];
  while (ds.search(addr)) {
Serial.print(" ROM =");
  for (int i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }
  Serial.write('\n');
  } 
  Serial.println(" No more addresses.");
  ds.reset_search();
  delay(2000);
 #endif


#ifdef DEBUG
//print temperature values for all sensors
for (int i = 0; i < NbSensor; i++) {
String mess = "Sensor ";
mess += i;
mess += " ";   
Serial.print(mess); 
printTemperature(sensorAdd[i]);
}
#endif
 
}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  
  Serial.print("temp (C): ");
  Serial.print(tempC);
  Serial.print("\n");
}

// fetch all temperature measurments into the array tempMeasurments
void returnAllTemperature(float temp[])
{
   for (int i = 0; i < NbSensor; i++) {
   float tempC = sensors.getTempC(sensorAdd[i]);
   temp[i] = tempC;
    #ifdef DEBUG
    String mess = "Sensor ";
    mess+=i;
    mess+=" ";
    mess+=temp[i];
    mess+="-";
    Serial.print(mess);
     #endif
   }
   #ifdef DEBUG
   Serial.print("\n");
   #endif
     
}
