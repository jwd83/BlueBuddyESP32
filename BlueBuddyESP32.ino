/**
   bluebuddy accessory
   reformat code is Ctrl+T
*/


/**
   include files
*/
#include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino


/**
   constants/pin definitions

   https://docs.iot-bus.com/en/latest/boards/iot-bus-io.html

   IOT-Bus Pin Description
   5 On-board LED
*/
#define PIN_LED 5
#define SERIAL_BAUD 115200


/**
   globals
*/

BluetoothSerial ESP_BT; //Object for Bluetooth


/**
   SETUP
   turn on bluetooth, wait for connection?
*/

void setup() {

  // setup our pins
  pinMode(PIN_LED, OUTPUT);


  // turn LED solid when booting
  ledOn();


  // turn on serial
  Serial.begin(115200);
  Serial.println("Serial....started");
  Serial.print("Bluetooth...");

  // start pairing mode
  ESP_BT.begin("BlueBuddy");
  Serial.print("started");

  // begin to toggle LED to indiciate pairing
  ledOff();

}

/**
   LOOP
   check for connection?
   send & receive commands
*/
void loop() {
  // put your main code here, to run repeatedly:
  delay(50);
  Serial.print("Tick!");
  ledToggle();


  delay(50);
  Serial.println("Tock!");
  ledToggle();
}

/**
   helper functions
*/

void ledToggle() {
  digitalWrite(PIN_LED, !digitalRead(PIN_LED));
}

void ledOn() {
  digitalWrite(PIN_LED, HIGH);
}

void ledOff() {
  digitalWrite(PIN_LED, LOW);
}

