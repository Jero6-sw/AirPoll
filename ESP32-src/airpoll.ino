/*
  AirPoll

  Board ESP32-WROOM-32

  Shield pollution management with :
   - capteur NO2 (TGS2611)
   - temperature (DS18B20)
   - Dust 1um et 2.5um (DSM501A)

  Bluetooth Communication with Aircasting application.

  modified 22/11/2020 by J.Galy
  
 

 ---- Note : Aircasting protocol

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

String Version = "03a";


#include <OneWire.h> 
#include <DallasTemperature.h>
#include "BluetoothSerial.h"
#include "esp_bt_device.h"
#include "Strings.h"

#include <Wire.h>
#include <ADS1115_WE.h>
#define I2C_ADDRESS 0x48


String aStr;

/********************************************************************/
// Pins assignment
/********************************************************************/
const int ledPin =  26;// the number of the LED pin
const int dustPin1 =  33; // DSM501A output 1 (>2.5um particules)
const int dustPin2 =  32; // DSM501A output 2 (>1.0um particules)
const int Analog0 =  34; // TGS2611 analog input
#define ONE_WIRE_BUS 25
// I²C SDA = GPIO21 for ADS1115
// I²C SCL = GPIO22 for ADS1115



/********************************************************************/
// ADS1115 ADC 4 lanes variables and instances
/********************************************************************/
ADS1115_WE adc(I2C_ADDRESS);
// ADS1115_WE adc = ADS1115_WE(); // Alternative: uses default address 0x48

float voltage=0.0;
int rawResult;


/********************************************************************/
// Bluetooth
/********************************************************************/
BluetoothSerial SerialBT;

String BTAddress; // BT MAC address

/********************************************************************/
// Temperature : DS18B20 sensor through 1-Wire
/********************************************************************/

OneWire oneWire(ONE_WIRE_BUS); 

DallasTemperature sensors(&oneWire);

float aTemperature;


/********************************************************************/
// LED variables
/********************************************************************/
int ledState = LOW;             // ledState used to set the LED

/********************************************************************/
// Dust variables
/********************************************************************/
volatile int dust_1 = 0;
volatile int dust_2 = 0;
volatile int dust_T = 0;
int  val_dust_PM1_0 = 0;
int  val_dust_PM2_5 = 0;

hw_timer_t *timer = NULL; // Timer for dust count
// It trigs every 1ms, counting the ratio between a low value of the dust pin
// and the total time -> percent value which can be converted into a dust quantity

/********************************************************************/
// Analog variables
/********************************************************************/
int AnalogVal = 0;           // variable to store the value read
unsigned long DurVal = 0;

/********************************************************************/
// Timing variables
/********************************************************************/
const unsigned long CINTERVAL=10; // seconds between 2 saves

unsigned long currentSavedMillis = 0;
unsigned long previousSavedMillis = 0;







// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// Sub funstions, Interrupt functions
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------

/********************************************************************/
// BT event
/********************************************************************/
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected");
  }

  if(event == ESP_SPP_CLOSE_EVT ){
    Serial.println("Client disconnected");
  }
}

/********************************************************************/
// BT address
/********************************************************************/
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

/********************************************************************/
// Dust interruption rotine (timer 1ms)
/********************************************************************/
void interrupt_dust() {

    dust_T=dust_T+1;
    if (digitalRead(dustPin1) == LOW)
       dust_1=dust_1+1;
    if (digitalRead(dustPin2) == LOW)
       dust_2=dust_2+1;
}


/********************************************************************/
// ADS1115 read analog lane
/********************************************************************/
float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  while(adc.isBusy()){}
  voltage = adc.getResult_mV();
  rawResult = adc.getRawResult();
  return voltage;
}

/********************************************************************/
// Sensors conversion functions
/********************************************************************/
// DSM501A theory, ratio is between 10000 and 14000 (low ratio -> ug/m3)
int conv_DSM501A_PM2_5(int dust_low, int dust_Total) {
  float particule_concentration = (float)(dust_low)*12000.0/(float)(dust_Total);
  return (int)(particule_concentration);
}

// DSM501A theory, ratio is between 10000 and 14000 (low ratio -> ug/m3)
int conv_DSM501A_PM1_0(int dust_low, int dust_Total) {
  float particule_concentration = (float)(dust_low)*12000.0/(float)(dust_Total);
  return (int)(particule_concentration);
}


// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// Setup
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------

void setup() {

   aStr = String();

   // initialize GPIOs
   pinMode(ledPin, OUTPUT);

   pinMode(dustPin1, INPUT);
   pinMode(dustPin2, INPUT);

   // I2C initialization (ADS1115)
   Wire.begin();

   // Debug port COM
   Serial.begin(115200);

  aStr = "Module AirPoll. Test v" + Version;
   Serial.println(aStr);
   Serial.println("------------------------");

  SerialBT.register_callback(callback);

  if(!SerialBT.begin("AirPoll")){
    Serial.println("An error occurred initializing Bluetooth");
  }else{
    BTAddress=getBTDeviceAddress();
    Serial.println("Bluetooth initialized (" + BTAddress + ")");
    
  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &interrupt_dust, true);  //attach callback
  timerAlarmWrite(timer, 1000, true); // Time in microseconds
  timerAlarmEnable(timer);

  if(!adc.init()){
    Serial.println("ADS1115 not connected!");
  }

  /* Set the voltage range of the ADC to adjust the gain
   * Please note that you must not apply more than VDD + 0.3V to the input pins!
   * 
   * ADS1115_RANGE_6144  ->  +/- 6144 mV
   * ADS1115_RANGE_4096  ->  +/- 4096 mV
   * ADS1115_RANGE_2048  ->  +/- 2048 mV (default)
   * ADS1115_RANGE_1024  ->  +/- 1024 mV
   * ADS1115_RANGE_0512  ->  +/- 512 mV
   * ADS1115_RANGE_0256  ->  +/- 256 mV
   */
  adc.setVoltageRange_mV(ADS1115_RANGE_2048); //comment line/change parameter to change range

  /* Set the inputs to be compared
   *  
   *  ADS1115_COMP_0_1    ->  compares 0 with 1 (default)
   *  ADS1115_COMP_0_3    ->  compares 0 with 3
   *  ADS1115_COMP_1_3    ->  compares 1 with 3
   *  ADS1115_COMP_2_3    ->  compares 2 with 3
   *  ADS1115_COMP_0_GND  ->  compares 0 with GND
   *  ADS1115_COMP_1_GND  ->  compares 1 with GND
   *  ADS1115_COMP_2_GND  ->  compares 2 with GND
   *  ADS1115_COMP_3_GND  ->  compares 3 with GND
   */
  //adc.setCompareChannels(ADS1115_COMP_0_GND); //uncomment if you want to change the default

  /* Set number of conversions after which the alert pin will be active
   * - or you can disable the alert 
   *  
   *  ADS1115_ASSERT_AFTER_1  -> after 1 conversion
   *  ADS1115_ASSERT_AFTER_2  -> after 2 conversions
   *  ADS1115_ASSERT_AFTER_4  -> after 4 conversions
   *  ADS1115_DISABLE_ALERT   -> disable comparator / alert pin (default) 
   */
  //adc.setAlertPinMode(ADS1115_ASSERT_AFTER_1); //uncomment if you want to change the default

  /* Set the conversion rate in SPS (samples per second)
   * Options should be self-explaining: 
   * 
   *  ADS1115_8_SPS 
   *  ADS1115_16_SPS  
   *  ADS1115_32_SPS 
   *  ADS1115_64_SPS  
   *  ADS1115_128_SPS (default)
   *  ADS1115_250_SPS 
   *  ADS1115_475_SPS 
   *  ADS1115_860_SPS 
   */
  //adc.setConvRate(ADS1115_8_SPS); //uncomment if you want to change the default

  /* Set continuous or single shot mode:
   * 
   *  ADS1115_CONTINUOUS  ->  continuous mode
   *  ADS1115_SINGLE     ->  single shot mode (default)
   */
  //adc.setMeasureMode(ADS1115_CONTINUOUS); //uncomment if you want to change the default

   /* Choose maximum limit or maximum and minimum alert limit (window)in Volt - alert pin will 
   *  be active when measured values are beyond the maximum limit or outside the window 
   *  Upper limit first: setAlertLimit_V(MODE, maximum, minimum)
   *  In max limit mode the minimum value is the limit where the alert pin will be deactivated (if 
   *  not latched)  
   * 
   *  ADS1115_MAX_LIMIT
   *  ADS1115_WINDOW
   * 
   */
  //adc.setAlertModeAndLimit_V(ADS1115_MAX_LIMIT, 3.0, 1.5); //uncomment if you want to change the default
  
  /* Enable or disable latch. If latch is enabled the alarm pin will be active until the
   * conversion register is read (getResult functions). If disabled the alarm pin will be
   * deactivated with next value within limits. 
   *  
   *  ADS1115_LATCH_DISABLED (default)
   *  ADS1115_LATCH_ENABLED
   */
  //adc.setAlertLatch(ADS1115_LATCH_ENABLED); //uncomment if you want to change the default

  /* Sets the alert pin polarity if active:
   *  
   * Enable or disable latch. If latch is enabled the alarm pin will be active until the
   * conversion register is read (getResult functions). If disabled the alarm pin will be
   * deactivated with next value within limits. 
   *  
   * ADS1115_ACT_LOW  ->  active low (default)   
   * ADS1115_ACT_HIGH ->  active high
   */
  //adc.setAlertPol(ADS1115_ACT_LOW); //uncomment if you want to change the default
 
  /* With this function the alert pin will be active, when a conversion is ready.
   * In order to deactivate, use the setAlertLimit_V function  
   */
  //adc.setAlertPinToConversionReady(); //uncomment if you want to change the default

  Serial.println("ADS1115 - Single Shot Mode");
   
  }
}





// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// Main Loop
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
void loop() {
   currentSavedMillis = millis();
   int i;

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
    Serial.print("A0=");
    Serial.print(AnalogVal);
    Serial.print("\t");

  voltage = readChannel(ADS1115_COMP_0_GND);
    Serial.print("ADS1115:");
    Serial.print(voltage);
  SerialBT.print(voltage-900.0);
    Serial.print("(");
    Serial.print(rawResult);
    Serial.print(")");
  voltage = readChannel(ADS1115_COMP_1_GND);
    Serial.print("-");
    Serial.print(voltage);
  voltage = readChannel(ADS1115_COMP_2_GND);
    Serial.print("-");
    Serial.print(voltage);
  voltage = readChannel(ADS1115_COMP_3_GND);
    Serial.print("-");
    Serial.print(voltage);
    Serial.print("\t");

  SerialBT.print(";AirPoll:" + BTAddress + ";AirPoll-CH4;Methane;CH4 Gas;response indicator;RI;0;25;50;75;100");
  SerialBT.print("\n");
    Serial.print("!");
    Serial.print(dust_1);
    Serial.print("-");
    Serial.print(dust_2);
    Serial.print("-");
    Serial.print(dust_T);
    Serial.print("\t");

  val_dust_PM2_5 = 100*dust_1/dust_T;
  val_dust_PM1_0 = 100*dust_2/dust_T;
//  SerialBT.print(val_dust_PM1_0);
//  SerialBT.print(";AirPoll:" + BTAddress + ";AirPoll-PM1;Particulate Matter;PM;response indicator;RI;0;25;50;75;100");
//  SerialBT.print("\n");
    Serial.print("PM1=");
    Serial.print(val_dust_PM1_0);
    Serial.print("\t");

//   SerialBT.print(val_dust_PM2_5);
//  SerialBT.print(";AirPoll:" + BTAddress + ";AirPoll-PM2.5;Particulate Matter;PM;response indicator;RI;0;25;50;75;100");
//  SerialBT.print("\n");
    Serial.print("PM2.5=");
    Serial.print(val_dust_PM2_5);
    Serial.print("\t");

  SerialBT.print(conv_DSM501A_PM1_0(dust_2,dust_T));
  SerialBT.print(";AirPoll:" + BTAddress + ";AirPoll-PM1;Particulate Matter;PM;micrograms per cubic meter;µg/m³;0;12;35;55;150");
  SerialBT.print("\n");
  SerialBT.print(conv_DSM501A_PM2_5(dust_1,dust_T));
  SerialBT.print(";AirPoll:" + BTAddress + ";AirPoll-PM2.5;Particulate Matter;PM;micrograms per cubic meter;µg/m³;0;12;35;55;150");
  SerialBT.print("\n");
      dust_T=0;
      dust_1=0;
      dust_2=0;


      sensors.requestTemperatures(); // Send the command to get temperature readings 
      //SavedTmpVal[indexSaved] = byte(10*sensors.getTempCByIndex(0)-100);
      aTemperature=sensors.getTempCByIndex(0);
  SerialBT.print((int)aTemperature);
  SerialBT.print(";Airpoll:" + BTAddress + ";AirPoll-C;Temperature;C;degrees Celsius;C;-5;5;15;25;35");
  SerialBT.print("\n");
    Serial.print("T=");
    Serial.print(aTemperature);
    Serial.print("°C\r");
      }

  
    if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }

  delay(20);

}
