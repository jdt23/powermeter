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

const float Radius = 0.17; //meters

int counter = 0;

const int DEBUG = 0;

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = A1;
const int LOADCELL_SCK_PIN = A0;

HX711 scale;
float accelerationSampleRate, gyroscopeSampleRate;

const int BATTERY_PIN = A2;

enum MeasurementMethod {AVERAGE, MAX, NONE};

struct Measurement {
  float accumulator;
  int samples;
  unsigned long last_time;
  unsigned long next_time;
  BLEUnsignedCharCharacteristic bleuchar;
  unsigned int refreshInterval; // in milliseconds
  MeasurementMethod method;

  Measurement(BLEUnsignedCharCharacteristic _bleuchar, const unsigned int _refreshInterval, MeasurementMethod _method) :
    accumulator(0),
    samples(0),
    last_time(0),
    next_time(0),
    bleuchar(_bleuchar),
    refreshInterval(_refreshInterval),
    method(_method)
    {}
  void reset() {
    accumulator = 0;
    samples = 0;
    last_time = 0;
    next_time = 0;
  }
  void addMeasurement(float a) {
    if (method==AVERAGE)
      accumulator += a;
    else if (method == MAX)
      accumulator = fmaxf(a, accumulator);
    samples++;
  }
  bool canGetMeasurement() {
    return (millis() > next_time);
  }
  float getMeasurement() {
    if (DEBUG) {
      Serial.print("accumulator=");
      Serial.print(accumulator);
      Serial.print(" samples=");
      Serial.println(samples);
    }
    float retval = NAN;
    if (method==AVERAGE)
      retval= accumulator / samples;
    else if (method==MAX)
      retval = accumulator;
    last_time = millis();
    next_time = last_time + refreshInterval;
    accumulator = 0;
    samples = 0;
    return retval;
  }
};
Measurement powerMeasurement(powerChar, 2000, MAX);
Measurement resistanceMeasurement(resistanceChar, 2000, MAX);
Measurement cadenceMeasurement(cadenceChar, 2000, AVERAGE);

void setLED (int r, int g, int b) {
  digitalWrite(LEDR, 1-r);
  digitalWrite(LEDG, 1-g);
  digitalWrite(LEDB, 1-b);
}

void setup() {
  if (DEBUG) Serial.begin(9600);
  //HX711 setup
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_offset(136200);
  // 1 pound = 4.4482216153 Newtons
  // scale factor = reading / (N) = reading / (lbs * 4.4482216153)
  //scale.set_scale(1);
  scale.set_scale(5000/ (5 * 4.4482216153));

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  setLED(0,0,1);

  //IMU setup
  while (!IMU.begin()) {
    if (DEBUG) Serial.println("Failed to initialize IMU!");
    setLED(1,1,0); // yellow
    delay(1000);
  }

  accelerationSampleRate = IMU.accelerationSampleRate();
  gyroscopeSampleRate    = IMU.gyroscopeSampleRate();

  //bluetooth setup
  while ( !BLE.begin() ) {
    if (DEBUG) Serial.println("starting BLE failed!");
    setLED(1,1,1); // white
    delay(1000);
  } 
  if (DEBUG) Serial.println ("Bluetooth startup succeeded");

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

  // https://www.reddit.com/r/pelotoncycle/comments/gwpyfw/diy_peloton_resistance_output/
  //  $Resistance = (145*($Power/(11.29*($Cadence-22.5)^1.25))^(.4651))
  float r = (145*pow(power/(11.29f*pow((rpm-22.5f),1.25f)),(0.4651)));
  
  // https://www.reddit.com/r/pelotoncycle/wiki/index/faq/bikecalibration
  
  // https://www.reddit.com/r/pelotoncycle/comments/b0bulz/how_the_peloton_bike_calculates_output_and_speed/
  // Output ~= (Cadence - 35) * (Resistance/100)2.5 * 24
  // Speed ~= (Cadence - 35)0.4 * (Resistance/100) * 9 + 0.4
  // Output ~= Speed2.5 / 10

  // Output ~= (Cadence - 35) * (Resistance/100)2.5 * 24
  // (Resistance/100)2.5 = Output / (24 * (Cadence-35))
  // Resistance = 100 * Output / (2.5*24*(cadence-35))
  //float r = 5 * power / (3 * (rpm - 35));
  
  return fmaxf(r,0);
}

unsigned char float2uchar (const float &val, const char * name, const float & maxval) {
  unsigned char val_uchar = (unsigned char)fabsf(val);
  if (DEBUG) {
    Serial.print(name);
    Serial.print("=");
    Serial.print(val_uchar);
    Serial.print(",");
    Serial.print(val);
    Serial.print("\t");
  }
  return min(val_uchar, maxval);
}

void loop() {

  setLED(1,0,0); // red
  delay(100);

  //bluetooth stuff
  BLEDevice central = BLE.central();

  if (central || DEBUG) {
    setLED(1,1,0); // yellow means central active but not connected yet

    if (DEBUG) Serial.print("Connected to central: ");
    if (DEBUG) Serial.print(central.address());

    unsigned char power_uchar=0, power_uchar_new=0;
    unsigned char resistance_uchar=0, resistance_uchar_new=0;
    unsigned char cadence_uchar=0, cadence_uchar_new=0;

    while (central.connected() || DEBUG) {
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

      // gyro_X is in degrees/second.  one revolution is 360 degrees.
      // rev/min = (deg/sec) * (60sec/1min) * (1rev/360deg)
      // rev/min = (deg/sec) * (1/6)
      //float angular_velocity = sqrtf(gyro_x*gyro_x + gyro_y*gyro_y + gyro_z*gyro_z);
      float angular_velocity = fabsf(gyro_z); // this is what it used to be, when i needed the 1.15x
      float new_cadence = (angular_velocity) / 6.0f;
      new_cadence *= 1.1667; // calibration factor. need to follow up here for lower cadences
      cadenceMeasurement.addMeasurement(new_cadence);

      /* Get values from the strain gauges */
      float reading = scale.get_units(1);
      if (DEBUG) {
        Serial.print("Scale Reading:\t"); Serial.println(reading);
      }
      // scale gives you force in newtons
      // Force_in_Newtons * angular velocity = instantaneous power
      // power (Newton-meter) = torque (radius(m)*Force*())* angular_velocity (in rad/sec)
      //   angular_velocity should be in radians/second
      //   rad/sec = (deg/sec) * (3.14159/180)(rad/deg)
      // 1Watt = 1Newton-meter per second
      float new_power = fabsf( Radius * reading * angular_velocity*3.14159/180  ) ;
      powerMeasurement.addMeasurement(new_power);

      float new_resistance = computeResistance(new_power, new_cadence);
      resistanceMeasurement.addMeasurement(new_resistance);

      // get new measurements if it's time to
      if (powerMeasurement.canGetMeasurement()) {
        power_uchar_new = float2uchar(powerMeasurement.getMeasurement(), "Power", 255);
      }
      if (resistanceMeasurement.canGetMeasurement()) {
        resistance_uchar_new = float2uchar(resistanceMeasurement.getMeasurement(), "Resistance", 99);
      }
      if (cadenceMeasurement.canGetMeasurement()) {
        cadence_uchar_new = float2uchar(cadenceMeasurement.getMeasurement(), "Cadence", 250);
      }

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
  delay(1000); // wait a second if not connected

  if (DEBUG) {
    int battery = analogRead(BATTERY_PIN);
    int batteryLevel = map(battery, 0, 1023, 0, 100);
    Serial.print("Battery: "); 
    Serial.print(batteryLevel);
    Serial.println("\%");
  }

  //Serial.println("");
  return;

}
