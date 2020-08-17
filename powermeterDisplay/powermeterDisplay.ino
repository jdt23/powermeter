#include <SevSeg.h>
SevSeg sevseg;

#include <ArduinoBLE.h>

int counter = 0;

const int DEBUG = 0;

void displayPowerResistanceCadence (unsigned char power, unsigned char resistance, unsigned char cadence) {
  /*
   * Product Name: LED Digital Display Tube; 
   * Type: Common Cathode; Model: 3631AH
   * Common Cathode: 12-9-8; Digital Display: 3 Digit; Digital Number: 3 Bit 7 Segment; Emitted Color: Red
   * Pin Number: 11; Continuous Forward Current: 20mA; Average Forward Voltage: 2V; Power Consumption: 36mW
   */

  sevseg.setNumber(1000000*power + 1000*resistance + cadence);

}


void setup() {
  if (DEBUG) Serial.begin(9600);

  byte segmentPins[] = {A0, 2,A7,A3,A2,A1,A6};

  byte numDigits = 9;
  byte digitPins[] = { 3, 4, 5,  // power pins
                      11, 6, 7,  // resistance pins
                      8, 9, 10}; // cadence pins
  
  bool resistorsOnSegments = true; 
  bool updateWithDelaysIn = true;
  byte hardwareConfig = COMMON_CATHODE; 
  bool updateWithDelays = false; // Default 'false' is Recommended
  bool leadingZeros = true; // Use 'true' if you'd like to keep the leading zeros
  bool disableDecPoint = true; // Use 'true' if your decimal point doesn't exist or isn't connected. Then, you only need to specify 7 segmentPins[]

  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments, updateWithDelays, leadingZeros, disableDecPoint);
  sevseg.setBrightness(90);
  
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

  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    if (DEBUG) Serial.print("Bluetooth connected to ");
    if (DEBUG) Serial.print(peripheral.localName());
    
    if (peripheral.localName() != "powerMeterService") {
      if (DEBUG) Serial.println("Wrong Local Name. returning.");
      return;
    }

    // stop scanning
    BLE.stopScan();

    if (peripheral.connect()) {
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
      
      unsigned char power=68, resistance=69, cadence=70;
      unsigned char newPower=68, newResistance=69, newCadence=70;
      int powerBytesRead=7, resistanceBytesRead=8, cadenceBytesRead=9;

      powerChar.readValue(power);
      resistanceChar.readValue(resistance);
      cadenceChar.readValue(cadence);
      
      while (peripheral.connected()) {
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
      
        if (DEBUG) Serial.print(" Bytes read: "); Serial.print(powerBytesRead); Serial.print(resistanceBytesRead); Serial.println(cadenceBytesRead);
        
        displayPowerResistanceCadence (power, resistance, cadence);
        if (DEBUG) {
          Serial.print(" P="); Serial.print(power);
          Serial.print(", R="); Serial.print(resistance);
          Serial.print(", C="); Serial.println(cadence);
        }
      }
    } else {
      if (DEBUG) Serial.println("Could not connect.");
    }

    // peripheral disconnected, start scanning again
    BLE.scanForUuid("1818");
  }
  else {  //if (peripheral) 
    displayPowerResistanceCadence(0,0,0);

    if (DEBUG) Serial.println("bluetooth not connected.");
  }
  
  //delay(100);
  sevseg.refreshDisplay(); 

}
