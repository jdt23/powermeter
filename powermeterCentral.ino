#include <ArduinoBLE.h>

// https://github.com/sigvaldm/SevenSeg
#include <SevenSeg.h>
//                           a, b, c, d, e, f, g
SevenSeg      powerDisplay (A6,A7,10, 9, 8, 7, 6);
SevenSeg resistanceDisplay (A6,A7,10, 9, 8, 7, 6);
SevenSeg    cadenceDisplay (A6,A7,10, 9, 8, 7, 6);
// the display i bought:    11, 7, 4, 2, 1,10, 5 (3 is DP), selects are 12, 9, 8

const int numOfDigits=3;
int      digitPinsPower[numOfDigits]={A0,A1,A2};
int digitPinsResistance[numOfDigits]={0 , 2,A5};
int    digitPinsCadence[numOfDigits]={ 3, 4, 5};

float displayFrequency = 100;
float digitDelay = 1000000.0f / (3.0f * numOfDigits * displayFrequency);
//8 displays, so T = 1/nF = 1/(8*100) = 1250 us
//float digitDelay = 1250;

int counter = 0;

void displayPowerResistanceCadence (int power, int resistance, int cadence) {


  /*
   * Product Name: LED Digital Display Tube; 
   * Type: Common Cathode; Model: 3631AH
   * Common Cathode: 12-9-8; Digital Display: 3 Digit; Digital Number: 3 Bit 7 Segment; Emitted Color: Red
   * Pin Number: 11; Continuous Forward Current: 20mA; Average Forward Voltage: 2V; Power Consumption: 36mW
   */

  powerDisplay.write(power);
  powerDisplay.clearDisp();

  resistanceDisplay.write(resistance);
  resistanceDisplay.clearDisp();

  cadenceDisplay.write(cadence);
  cadenceDisplay.clearDisp();

}


void setup() {
  
  powerDisplay.setDigitPins(numOfDigits, digitPinsPower);
  powerDisplay.setDigitDelay(digitDelay);
  powerDisplay.setCommonCathode();
  
  resistanceDisplay.setDigitPins(numOfDigits, digitPinsResistance);
  resistanceDisplay.setDigitDelay(digitDelay);
  resistanceDisplay.setCommonCathode();

  cadenceDisplay.setDigitPins(numOfDigits, digitPinsCadence);
  cadenceDisplay.setDigitDelay(digitDelay);
  cadenceDisplay.setCommonCathode();

  //Serial.begin(9600);
  //while (!Serial)
  //  ;

  //Serial.println("BLE Central - powermeterCentral");

  // initialize the BLE hardware
  BLE.begin();

  // start scanning for peripherals
  BLE.scanForUuid("1818");
}

void loop() {
  //Serial.println("Entering loop");

  //displayPowerResistanceCadence (987,654,321);
  //return;

  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    if (peripheral.localName() != "powerMeterService") {
      return;
    }

    // stop scanning
    BLE.stopScan();

    //controlLed(peripheral);
    if (peripheral.connect()) {
      Serial.println("Connected");
    } else {
      Serial.println("Failed to connect!");
      return;
    }

    BLEUnsignedCharCharacteristic powerChar("2A63", BLERead | BLENotify);
    BLEUnsignedCharCharacteristic resistanceChar("2AD6", BLERead | BLENotify);
    BLEUnsignedCharCharacteristic cadenceChar("2A5B", BLERead | BLENotify);
    int power = (int)powerChar.value();
    int resistance = (int)resistanceChar.value();
    int cadence = (int)cadenceChar.value();
    
    /*
    Serial.print("Power=");      Serial.print(power);      Serial.println("");
    Serial.print("Resistance="); Serial.print(resistance); Serial.println("");
    Serial.print("Cadence=");    Serial.print(cadence);    Serial.println(""); 
     */

    //displayPowerResistanceCadence (power, resistance, cadence);
    
    // peripheral disconnected, start scanning again
    BLE.scanForUuid("1818");
  }
  else {
    
    powerDisplay.write("---");
    powerDisplay.clearDisp();

    resistanceDisplay.write("---");
    resistanceDisplay.clearDisp();

    cadenceDisplay.write("---");
    cadenceDisplay.clearDisp();
    
  }
  
}
