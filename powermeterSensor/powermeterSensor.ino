// HX711 Arduino Library - Version: Latest
// For strain gauges
#include <HX711.h>

// ArduinoBLE - Version: Latest
// For bluetooth
#include <ArduinoBLE.h>

BLEService powerMeterService("1818");
BLEUnsignedCharCharacteristic powerChar("2A63", BLERead | BLENotify );
BLEUnsignedCharCharacteristic resistanceChar("2AD6", BLERead | BLENotify );
BLEUnsignedCharCharacteristic cadenceChar("2A5B", BLERead | BLENotify );

// Arduino_LSM9DS1 - Version: Latest
// For IMU
#include <Arduino_LSM9DS1.h>

int counter = 0;

const int DEBUG = 0;

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = A7;
const int LOADCELL_SCK_PIN = A6;

HX711 scale;
float offset, calibrationFactor;
float accelerationSampleRate, gyroscopeSampleRate;

//unsigned long previousMillis = 0;

void setup() {
  if (DEBUG) Serial.begin(9600);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  if (!IMU.begin()) {
    if (DEBUG) Serial.println("Failed to initialize IMU!");
    while (1);
  }

  offset = 0;
  calibrationFactor = 0;

  accelerationSampleRate = IMU.accelerationSampleRate();
  gyroscopeSampleRate    = IMU.gyroscopeSampleRate();

  //bluetooth setup
  if ( !BLE.begin() ) {
    if (DEBUG) Serial.println("starting BLE failed!");
    while (1);
  } else {
    if (DEBUG) Serial.println ("Bluetooth startup succeeded");
  }

  BLE.setLocalName("powerMeterService");
  BLE.setAdvertisedService(powerMeterService);
  powerMeterService.addCharacteristic(powerChar);
  powerMeterService.addCharacteristic(resistanceChar);
  powerMeterService.addCharacteristic(cadenceChar);
  BLE.addService(powerMeterService);

  BLE.advertise();
  if (DEBUG) Serial.println("Bluetooth device active, waiting for connections...");
}

float computeResistance ( const float &power, const float &rpm ) {

  // from https://www.reddit.com/r/pelotoncycle/comments/b0bulz/how_the_peloton_bike_calculates_output_and_speed/
  // Output ~= (Cadence - 35) * (Resistance/100)2.5 * 24
  // Speed ~= (Cadence - 35)0.4 * (Resistance/100) * 9 + 0.4
  // Output ~= Speed2.5 / 10

  // Output ~= (Cadence - 35) * (Resistance/100)2.5 * 24
  // (Resistance/100)2.5 = Output / (24 * (Cadence-35))
  // Resistance = 100 * Output / (2.5*24*(cadence-35))
  float r = 5 * power / (3 * (rpm - 35));
  return r;
}

unsigned char float2uchar (const float &val, const char * name) {
  unsigned char val_uchar = (unsigned char)fabsf(val);
  if (DEBUG) {
    Serial.print(name);
    Serial.print("=");
    Serial.print(val_uchar);
    Serial.print(",");
    Serial.print(val);
    Serial.print("\t");
  }
  return val_uchar;
}

void loop() {

  delay(1000);

  //bluetooth stuff
  BLEDevice central = BLE.central();

  if (central) {
  
    if (DEBUG) Serial.print("Connected to central: ");
    if (DEBUG) Serial.print(central.address());

    unsigned char power_uchar=0, power_uchar_new=0;
    unsigned char resistance_uchar=0, resistance_uchar_new=0;
    unsigned char cadence_uchar=0, cadence_uchar_new=0;
    
    while (central.connected()) {
      
      if (DEBUG) Serial.print("\nCounter = ");
      if (DEBUG) Serial.print(counter++);
    
      // get gyro_z
      float gyro_x, gyro_y, gyro_z = 0;
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyro_x, gyro_y, gyro_z);

        if (DEBUG) {
          Serial.print("\tGyro=\t");
          Serial.print(gyro_x);
          Serial.print('\t');
          Serial.print(gyro_y);
          Serial.print('\t');
          Serial.print(gyro_z);
        }
      } else {
        if (DEBUG) Serial.print("gyro not available\t");
      }
      if (DEBUG) Serial.print("\t");

//      float GyZ, GyZ_Radians, cadence;
//      GyZ = gyro_z;
//      /* Time to do some maths */
//      //GyZ_Radians = -GyZ * (1 / 32.8) * (3.14 / 180); // Convert into radians / sec
//      cadence = -GyZ * 0.0051; // Convert into revs / sec

      // gyro_z is in degrees/second.  one revolution is 360 degrees.
      // rev/min = (deg/sec) * (60sec/1min) * (1rev/360deg)
      // rev/min = (deg/sec) * (1/6)
      float cadence = gyro_z / 6.0f;
      cadence *= 1.15; // calibration factor. need to follow up here for lower cadences
      
      /* Get values from the strain gauges */
      //float reading = scale.read();
      //Serial.print("Scale Reading:\t"); Serial.println(reading);
      float power = 100.0f; //XXX
    
      float resistance = computeResistance(power, cadence);
    
      power_uchar_new      = float2uchar(power, "Power");
      cadence_uchar_new    = float2uchar(cadence, "Cadence");
      resistance_uchar_new = float2uchar(resistance, "Resistance");

      // only write new values if needed
      if (power_uchar_new != power_uchar) {
        power_uchar = power_uchar_new;
        powerChar.writeValue(power_uchar);
      }
      if (resistance_uchar_new != resistance_uchar) {
        resistance_uchar = resistance_uchar_new;
        resistanceChar.writeValue(resistance_uchar);
      }
      if (cadence_uchar_new != cadence_uchar) {
        cadence_uchar = cadence_uchar_new;
        cadenceChar.writeValue(cadence_uchar);
      }
      
      // add delay to prevent values from changing too quickly
      delay(500);
    }
  }
  //Serial.println("");
  return;

}
