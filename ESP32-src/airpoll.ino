/*
  Pollution v02

  Gestion shield pollution avec :
   - capteur NO2 (Ain0)
   - temperature (DS18B20 on In2)
   - Dust (DSM501A on In3-4)

  modified x xxx 201x
  by 
  
 
  Communication bluetooth avec un ESP32.

*/
/*
Each sensor reading should be written as one line to the serial output. Lines should end
with '\n' and should have the following format:

<Measurement value>;<Sensor package name>;<Sensor name>;<Type of measurement>;<Short type of measurement>;<Unit name>;<Unit symbol/abbreviation>;<T1>;<T2>;<T3>;<T4>;<T5>

The Sensor name should be different for each sensor.

T1..T5 are integer thresholds which guide how values should be displayed -
- lower than T1 - extremely low / won't be displayed
- between T1 and T2 - low / green
- between T2 and T3 - medium / yellow
- between T3 and T4 - high / orange
- between T4 and T5 - very high / red
- higher than T5 - extremely high / won't be displayed
*/


#include <OneWire.h> 
#include <DallasTemperature.h>
#include "BluetoothSerial.h"
#include "esp_bt_device.h"
#include "Strings.h"

//#include "Arduino.h"
//#include <ESP32AnalogRead.h>
//ESP32AnalogRead adc;


#define ONE_WIRE_BUS 25

BluetoothSerial SerialBT;

/********************************************************************/
// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(ONE_WIRE_BUS); 
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
/********************************************************************/ 

String BTAddress; // BT MAC address

// constants won't change. Used here to set a pin number:
const int ledPin =  26;// the number of the LED pin
const int dustPin1 =  33; // DSM501A output 1
const int dustPin2 =  32; // DSM501A output 2
const int Analog0 =  34; // TGS2611 analog input

// Variables will change:
int ledState = LOW;             // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
//const long interval = 1000;           // interval at which to blink (milliseconds)


unsigned long interval = 1000;           // interval at which to blink (milliseconds)

int AnalogVal = 0;           // variable to store the value read
unsigned long DurVal = 0;

const unsigned long CINTERVAL=10; // seconds between 2 saves
unsigned long previousSavedMillis = 0;

volatile int state = 0;


void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected");
  }

  if(event == ESP_SPP_CLOSE_EVT ){
    Serial.println("Client disconnected");
  }
}

String getBTDeviceAddress() {
 
  const uint8_t* point = esp_bt_dev_get_address();
  String Stmp;
 
  for (int i = 0; i < 6; i++) {
 
    char str[3];
 
    sprintf(str, "%02X", (int)point[i]);
    Stmp.concat(str);
 
    if (i < 5){
      Stmp.concat(":");
    }
  }
  return Stmp;
}
 
void setup() {
   // initialize digital pin LED_BUILTIN as an output.
   pinMode(ledPin, OUTPUT);

   pinMode(dustPin1, INPUT);
   pinMode(dustPin2, INPUT);

/*
  pinMode(Analog0,INPUT);
  adcAttachPin(Analog0);
  analogReadResolution(11);
  analogSetAttenuation(ADC_6db);
*/
//adc.attach(4);

  
   Serial.begin(115200);

   Serial.println("Module AirPoll. Test v02e");
   Serial.println("------------------------");

  SerialBT.register_callback(callback);

  if(!SerialBT.begin("AirPoll")){
    Serial.println("An error occurred initializing Bluetooth");
  }else{
    BTAddress=getBTDeviceAddress();
    Serial.println("Bluetooth initialized (" + BTAddress + ")");
    
   attachInterrupt(digitalPinToInterrupt(dustPin1), dust, FALLING);

   
  }
}


void dust() {
    state = state + 1;
}




void loop() {
   unsigned long currentMillis = millis();
   unsigned long currentSavedMillis = millis();
   int i;

/*
   // ---------------------------------------
   // Gestion LED
   if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;

      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW) {
         ledState = HIGH;
         interval = 50;
         }
      else {
         ledState = LOW;
         interval = 950;
         }

      // set the LED with the ledState of the variable:
      digitalWrite(ledPin, ledState);
      }
*/

   // ---------------------------------------
   // Gestion sauvegarde voie analogique
   if (currentSavedMillis - previousSavedMillis >= (CINTERVAL*long(1000))) {
      // save the last time you blinked the LED
      previousSavedMillis = currentSavedMillis;

      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW) {
         ledState = HIGH;
         }
      else {
         ledState = LOW;
         }
      // set the LED with the ledState of the variable:
      digitalWrite(ledPin, ledState);

      //SavedAnalogVal[indexSaved] = byte(analogRead(0));
  //SerialBT.print(adc.readVoltage()); //analogRead(Analog0));
  AnalogVal=analogRead(Analog0);
  SerialBT.print(AnalogVal);
    //Serial.print(AnalogVal);
    //Serial.print("$");

  SerialBT.print(";AirPoll:" + BTAddress + ";AirPoll-CH4;Methane;CH4 Gas;response indicator;RI;0;25;50;75;100");
  SerialBT.print("\n");
      //SavedDigVal[indexSaved] = byte(state);
  SerialBT.print(state);
  SerialBT.print(";AirPoll:" + BTAddress + ";AirPoll-PM1;Particulate Matter;PM;response indicator;RI;0;25;50;75;100");
  SerialBT.print("\n");
      state=0;
      sensors.requestTemperatures(); // Send the command to get temperature readings 
      //SavedTmpVal[indexSaved] = byte(10*sensors.getTempCByIndex(0)-100);
  SerialBT.print(sensors.getTempCByIndex(0));
  SerialBT.print(";Airpoll:" + BTAddress + ";AirPoll-C;Temperature;C;degrees Celsius;C;-5;5;15;25;35");
  SerialBT.print("\n");
      }

  
    if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }

  delay(20);

}
