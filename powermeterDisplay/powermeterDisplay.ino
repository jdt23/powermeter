// make sure to set MAXNUMDIGITS to 9 in your local copy. Default is 8 digits
// and defining it here doesn't seem to work
#include <SevSeg.h>
SevSeg sevseg;

#include <ArduinoBLE.h>

int counter = 0;

const int DEBUG = 0;

const int BATTERY_PIN = A4;

// inputs are binary ints. either 0 or 1.
// but need to invert from the way I expect since high is off and low is on
void setLED (int r, int g, int b) {
  digitalWrite(LEDR, 1-r);
  digitalWrite(LEDG, 1-g);
  digitalWrite(LEDB, 1-b);
}

// not using this function yet, as it screws up timing for the digits. 
void removeLeadingZeros(int val, int hundredsPin, int tensPin) {
  // common cathode, so HIGH is off
  if (val < 100) {
    digitalWrite(hundredsPin, HIGH);
  }
  if (val < 10) {
    digitalWrite(tensPin, HIGH);
  }
}

void displayPowerResistanceCadence (int power, int resistance, int cadence) {
  /*
   * Product Name: LED Digital Display Tube; 
   * Type: Common Cathode; Model: 3631AH
   * Common Cathode: 12-9-8; Digital Display: 3 Digit; Digital Number: 3 Bit 7 Segment; Emitted Color: Red
   * Pin Number: 11; Continuous Forward Current: 20mA; Average Forward Voltage: 2V; Power Consumption: 36mW
   */

  sevseg.setNumber(1000000*power + 1000*resistance + cadence);
//  removeLeadingZeros(power, 3, 4);
//  removeLeadingZeros(resistance, 11, 6);
//  removeLeadingZeros(cadence, 8, 9);
  sevseg.refreshDisplay();
}

void setup() {
  if (DEBUG) Serial.begin(9600);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  setLED(0,0,1);

  // set up LED
  byte segmentPins[] = {A0, 2,A7,A3,A2,A1,A6};
  byte numDigits = 9;
  byte digitPins[] = { 3, 4, 5,  // power pins
                      11, 6, 7,  // resistance pins
                      8, 9, 10}; // cadence pins
  bool resistorsOnSegments = true; 
  //bool updateWithDelaysIn = true;
  byte hardwareConfig = COMMON_CATHODE; 
  bool updateWithDelays = false; // Default 'false' is Recommended
  bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
  bool disableDecPoint = true; // Use 'true' if your decimal point doesn't exist or isn't connected. Then, you only need to specify 7 segmentPins[]
  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments, updateWithDelays, leadingZeros, disableDecPoint);
  sevseg.setBrightness(100);

  // initialize the BLE hardware
  if ( !BLE.begin() ) {
    if (DEBUG) Serial.println("starting BLE failed!");
    while (1);
  } else {
    if (DEBUG) Serial.println ("Bluetooth startup succeeded");
  }

  // start scanning for peripherals
  BLE.scanForUuid("1818");

  //BLE.debug(Serial);
}

void loop() {

  setLED(1,0,0); // red
  //delay(100);

  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    setLED(1,1,0); // yellow means peripheral active but not connected yet
    if (DEBUG) Serial.print("Bluetooth connected to ");
    if (DEBUG) Serial.print(peripheral.localName());

    if (peripheral.localName() != "powerMeterService") {
      if (DEBUG) Serial.println("Wrong Local Name. returning.");
      return;
    }

    // stop scanning
    BLE.stopScan();

    if (peripheral.connect()) {
      setLED(0,1,0);
      if (DEBUG) Serial.println (" Bluetooth connected.");

      // discover peripheral attributes
      if (DEBUG) Serial.println("Discovering attributes ...");
      if (peripheral.discoverAttributes()) {
        if (DEBUG) Serial.println("Attributes discovered");
      } else {
        if (DEBUG) Serial.println("Attribute discovery failed!");
        peripheral.disconnect();
        return;
      }

      //int characteristicCount = peripheral.characteristicCount();
      //Serial.print(characteristicCount);
      //Serial.println(" characteristics discovered");

      BLECharacteristic powerChar = peripheral.characteristic("2A63");
      BLECharacteristic resistanceChar = peripheral.characteristic("2AD6");
      BLECharacteristic cadenceChar = peripheral.characteristic("2A5B");

      // subscribe to the powerChar characteristic
      if (DEBUG) Serial.println("Subscribing to powerChar ...");
      if (!powerChar) {
        if (DEBUG) Serial.println("no powerChar found!");
        peripheral.disconnect();
        return;
      } else if (!powerChar.canSubscribe()) {
        if (DEBUG) Serial.println("powerChar is not subscribable!");
        peripheral.disconnect();
        return;
      } else if (!powerChar.subscribe()) {
        if (DEBUG) Serial.println("powerChar subscription failed!");
        peripheral.disconnect();
        return;
      }

      // subscribe to the resistanceChar characteristic
      if (DEBUG) Serial.println("Subscribing to resistanceChar ...");
      if (!resistanceChar) {
        if (DEBUG) Serial.println("no resistanceChar found!");
        peripheral.disconnect();
        return;
      } else if (!resistanceChar.canSubscribe()) {
        if (DEBUG) Serial.println("resistanceChar is not subscribable!");
        peripheral.disconnect();
        return;
      } else if (!resistanceChar.subscribe()) {
        if (DEBUG) Serial.println("resistanceChar subscription failed!");
        peripheral.disconnect();
        return;
      }

      // subscribe to the cadenceChar characteristic
      if (DEBUG) Serial.println("Subscribing to cadenceChar ...");
      if (!cadenceChar) {
        if (DEBUG) Serial.println("no cadenceChar found!");
        peripheral.disconnect();
        return;
      } else if (!cadenceChar.canSubscribe()) {
        if (DEBUG) Serial.println("cadenceChar is not subscribable!");
        peripheral.disconnect();
        return;
      } else if (!cadenceChar.subscribe()) {
        if (DEBUG) Serial.println("cadenceChar subscription failed!");
        peripheral.disconnect();
        return;
      }

      unsigned short power = 68, newPower = 68;
      unsigned char resistance = 69, newResistance = 69;
      unsigned char cadence = 70, newCadence = 70;
      int powerBytesRead = 7, resistanceBytesRead = 8, cadenceBytesRead = 9;

      powerChar.readValue(power);
      resistanceChar.readValue(resistance);
      cadenceChar.readValue(cadence);

      while (peripheral.connected()) {
        setLED(0,0,0);
        if (powerChar.valueUpdated()) {
          powerBytesRead = powerChar.readValue(newPower);
          power = newPower;
        }
        if (resistanceChar.valueUpdated()) {
          resistanceBytesRead = resistanceChar.readValue(newResistance);
          resistance = newResistance;
        }
        if (cadenceChar.valueUpdated()) {
          cadenceBytesRead = cadenceChar.readValue(newCadence);
          cadence = newCadence;
        }

        if (DEBUG) {
          Serial.print(" Bytes read: ");
          Serial.print(powerBytesRead);
          Serial.print(resistanceBytesRead);
          Serial.println(cadenceBytesRead);
        }

        displayPowerResistanceCadence (power, resistance, cadence);
        if (DEBUG) {
          Serial.print(" P="); Serial.print(power);
          Serial.print(", R="); Serial.print(resistance);
          Serial.print(", C="); Serial.println(cadence);
        }
      }
    } else {
      setLED(0,1,1);
      if (DEBUG) Serial.println("Could not connect.");
    }

    // peripheral disconnected, start scanning again
    BLE.scanForUuid("1818");
  }
  else {  //if (peripheral)
    if (DEBUG) Serial.println("bluetooth not connected.");

    //displayPowerResistanceCadence(123,456,789);

    int battery = analogRead(BATTERY_PIN);
    int batteryLevel = map(battery, 0, 1023, 0, 100);
    if (DEBUG) Serial.print("Battery: ");
    if (DEBUG) Serial.print(battery);
    if (DEBUG) Serial.print(", ");
    if (DEBUG) Serial.println(batteryLevel);
    displayPowerResistanceCadence(0,0,batteryLevel);
    setLED(1,1,0);

    //delay(100);
  }

  //delay(100);

}
