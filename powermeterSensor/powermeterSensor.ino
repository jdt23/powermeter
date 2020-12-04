// HX711 Arduino Library - Version: Latest
// For strain gauges
#include <HX711.h>

// ArduinoBLE - Version: Latest
// For bluetooth
#include <ArduinoBLE.h>

BLEService powerMeterService("1818");
BLEUnsignedShortCharacteristic powerChar("2A63", BLERead | BLENotify );
BLEUnsignedCharCharacteristic  resistanceChar("2AD6", BLERead | BLENotify );
BLEUnsignedCharCharacteristic  cadenceChar("2A5B", BLERead | BLENotify );

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
  unsigned int refreshInterval; // in milliseconds
  MeasurementMethod method;

  Measurement(const unsigned int _refreshInterval, MeasurementMethod _method) :
    accumulator(0),
    samples(0),
    last_time(0),
    next_time(0),
    refreshInterval(_refreshInterval),
    method(_method)
    {}
  void reset() {
    accumulator = 0;
    samples = 0;
    last_time = millis();
    next_time = last_time + refreshInterval;
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
    if (samples == 0)
      return NAN;
    float retval = NAN;
    if (method==AVERAGE)
      retval= accumulator / samples;
    else if (method==MAX)
      retval = accumulator;
    reset();
    return retval;
  }
};
Measurement powerMeasurement(2000, AVERAGE);
Measurement resistanceMeasurement(2000, AVERAGE);
Measurement cadenceMeasurement(2000, AVERAGE);

void setLED (int r, int g, int b) {
  digitalWrite(LEDR, 1-r);
  digitalWrite(LEDG, 1-g);
  digitalWrite(LEDB, 1-b);
}

void setup() {
  if (DEBUG) Serial.begin(9600);
  //HX711 setup
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  //scale.set_offset(1);
  if (DEBUG) Serial.println("Calibrating scale.");
  float offset = scale.get_units(20);
  offset = scale.get_units(10);
  if (DEBUG) Serial.print("Setting offset to ");
  if (DEBUG) Serial.println(offset);
  scale.set_offset(offset);
  //scale.set_offset(136200);
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
  float r = (145*pow(power/(11.29f*pow(max( (rpm-22.5f),1 ),1.25f)),(0.4651)));

  // https://www.reddit.com/r/pelotoncycle/wiki/index/faq/bikecalibration

  // https://www.reddit.com/r/pelotoncycle/comments/b0bulz/how_the_peloton_bike_calculates_output_and_speed/
  // Output ~= (Cadence - 35) * (Resistance/100)2.5 * 24
  // Speed ~= (Cadence - 35)0.4 * (Resistance/100) * 9 + 0.4
  // Output ~= Speed2.5 / 10

  // Output ~= (Cadence - 35) * (Resistance/100)2.5 * 24
  // (Resistance/100)2.5 = Output / (24 * (Cadence-35))
  // Resistance = 100 * Output / (2.5*24*(cadence-35))
  //float r = 5 * power / (3 * (rpm - 35));

  if (DEBUG) {
    Serial.print("computing resistance: p  =");  Serial.println(power);
    Serial.print("computing resistance: rpm=");  Serial.println(rpm);
    Serial.print("computing resistance: r  =");  Serial.println(r);
  }
  return fmaxf(r,0);
}

unsigned short float2ushort (const float &val, const char * name, const float & maxval) {
  unsigned short val_uchar = (unsigned short)fabsf(val+0.5f);// 0.5 for rounding
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

unsigned char float2uchar (const float &val, const char * name, const float & maxval) {
  unsigned char val_uchar = (unsigned char)fabsf(val+0.5f);// 0.5 for rounding
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

    unsigned short power_short=0, power_short_new=0;
    unsigned char  resistance_char=0, resistance_char_new=0;
    unsigned char  cadence_char=0, cadence_char_new=0;

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
      // HACK: not sure why we need to multiply by (7/6), but doing this matches my wahoo meter.
      new_cadence *= (7/6);
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
      float new_power = fabsf( Radius * fabsf(reading) * angular_velocity * 3.1415926535 / 180 );
      // HACK: divide power by 2. hand wave something about measuring power on one of two pedals
      new_power /= 2.0f;
      powerMeasurement.addMeasurement(new_power);

      // get new measurements if it's time to
      if (powerMeasurement.canGetMeasurement()) {
        power_short_new = float2ushort(powerMeasurement.getMeasurement(), "Power", 999);
      }
      // only write new values if needed
      if (power_short_new != power_short) {
        power_short = power_short_new;
        powerChar.writeValue(power_short);
      }

      if (cadenceMeasurement.canGetMeasurement()) {
        cadence_char_new = float2uchar(cadenceMeasurement.getMeasurement(), "Cadence", 250);
      }
      if (cadence_char_new != cadence_char) {
        cadence_char = cadence_char_new;
        cadenceChar.writeValue(cadence_char);
      }

      float new_resistance = computeResistance(power_short, cadence_char);
      resistanceMeasurement.addMeasurement(new_resistance);
      if (resistanceMeasurement.canGetMeasurement()) {
        resistance_char_new = float2uchar(resistanceMeasurement.getMeasurement(), "Resistance", 99);
      }
      if (resistance_char_new != resistance_char) {
        resistance_char = resistance_char_new;
        resistanceChar.writeValue(resistance_char);
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
