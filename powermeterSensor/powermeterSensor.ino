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

struct Measurement {
  float accumulator;
  int samples;
  unsigned long last_time;
  unsigned long next_time;
  BLEUnsignedCharCharacteristic bleuchar;
  const unsigned long refreshInterval = 1000; // in milliseconds

  Measurement(BLEUnsignedCharCharacteristic _bleuchar) :
    accumulator(0),
    samples(0),
    last_time(0),
    next_time(0),
    bleuchar(_bleuchar)
    {}
  void reset() {
    accumulator = 0;
    samples = 0;
    last_time = 0;
    next_time = 0;
  }
  void addMeasurement(float a) {
    accumulator += a;
    samples++;
  }
  bool canGetMeasurement() {
    return (millis() > next_time);
  }
  float getMeasurement() {
    float retval = accumulator / samples;
    last_time = millis();
    next_time = last_time + refreshInterval;
    accumulator = 0;
    samples = 0;
    return retval;
  }
};
Measurement powerMeasurement(powerChar);
Measurement resistanceMeasurement(resistanceChar);
Measurement cadenceMeasurement(cadenceChar);

void setLED (int r, int g, int b) {
  digitalWrite(LEDR, 1-r);
  digitalWrite(LEDG, 1-g);
  digitalWrite(LEDB, 1-b);
}

void setup() {
  if (DEBUG) Serial.begin(9600);
  //HX711 setup
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  setLED(0,0,1);

  //IMU setup
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

  setLED(1,0,0); // red
  delay(100);

  //bluetooth stuff
  BLEDevice central = BLE.central();

  if (central) {
    setLED(1,1,0); // yellow means central active but not connected yet

    if (DEBUG) Serial.print("Connected to central: ");
    if (DEBUG) Serial.print(central.address());

    unsigned char power_uchar=0, power_uchar_new=0;
    unsigned char resistance_uchar=0, resistance_uchar_new=0;
    unsigned char cadence_uchar=0, cadence_uchar_new=0;
    float cadence=0, resistance=0, power=0;
    int cadence_samples=0, resistance_samples=0, power_samples=0;


    while (central.connected()) {
      setLED(0,0,0); // turn leds off while connected

      if (DEBUG) Serial.print("\nCounter = ");
      if (DEBUG) Serial.print(counter++);

      // to measure cadence, the gyroscope Z dimension tells you degrees/second
      // which can easily be converted to RPM
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

      // gyro_z is in degrees/second.  one revolution is 360 degrees.
      // rev/min = (deg/sec) * (60sec/1min) * (1rev/360deg)
      // rev/min = (deg/sec) * (1/6)
      float new_cadence = gyro_z / 6.0f;
      new_cadence *= 1.15; // calibration factor. need to follow up here for lower cadences
      cadenceMeasurement.addMeasurement(new_cadence);

      /* Get values from the strain gauges */
      //float reading = scale.read();
      //Serial.print("Scale Reading:\t"); Serial.println(reading);
      float new_power = 100.0f; //XXX
      powerMeasurement.addMeasurement(new_power);

      float new_resistance = computeResistance(power, cadence);
      resistanceMeasurement.addMeasurement(new_resistance);

      if (powerMeasurement.canGetMeasurement()) {
        power_uchar_new = float2uchar(powerMeasurement.getMeasurement(), "Power");
      }
      if (resistanceMeasurement.canGetMeasurement()) {
        resistance_uchar_new = float2uchar(resistanceMeasurement.getMeasurement(), "Resistance");
      }
      if (cadenceMeasurement.canGetMeasurement()) {
        cadence_uchar_new = float2uchar(cadenceMeasurement.getMeasurement(), "Cadence");
      }
      //power_uchar_new      = float2uchar(power, "Power");
      //cadence_uchar_new    = float2uchar(cadence, "Cadence");
      //resistance_uchar_new = float2uchar(resistance, "Resistance");

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
      delay(100);
    }
    setLED(1,1,0); // yellow should only be on temporarily

  }
  setLED(1,0,0); // red means not connected

  //Serial.println("");
  return;

}
