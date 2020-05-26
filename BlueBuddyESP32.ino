/*
   bluebuddy accessory
   reformat code is Ctrl+T
*/


/*
   include files
*/
#include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino


/*
   constants/pin definitions

   https://docs.iot-bus.com/en/latest/boards/iot-bus-io.html

   IOT-Bus Pin Description
   5 On-board LED
*/
#define PIN_LED           5
#define SERIAL_BAUD       115200

#define MODE_BRIDGE     1
#define MODE_COMMAND    2

/*
   globals
*/
BluetoothSerial SerialBT; //Object for Bluetooth

uint32_t loop_mode = MODE_COMMAND;
uint32_t blink_last = 0;
uint32_t blink_rate = 1000;


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
  SerialBT.begin("BlueBuddy");
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
  switch (loop_mode) {
    case MODE_BRIDGE:
      bridge_serials();
      blink_led();
      break;
    case MODE_COMMAND:
      respond_to_command();
      break;

  }
}


void respond_to_command() {
  char in = 0;

  if (SerialBT.available()) {
    in = SerialBT.read();
    switch (in) {
      /*
         a turns the LED on
      */
      case 'a':
        ledOn();
        break;

      /*
         b turns the LED off
      */
      case 'b':
        ledOff();
        break;

      /*
         d and q disconnect/quit
      */
      case 'd':
      case 'q':
        // disconnect
        SerialBT.end();
        
        // todo investigate adding delay before restarting pairing. device may just reconnect.
        // using a bluetooth terminal this seems to work fine without delay.
        
        // begin pairing mode for next device
        SerialBT.begin("BlueBuddy");
        break;
    }
  }
}

void bridge_serials() {
  char sbt_read;


  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }

  if (SerialBT.available()) {
    sbt_read = SerialBT.read();
    Serial.write(sbt_read);

    switch (sbt_read) {
      case 'a':
        blink_rate = 50;
        break;

      case 'b':
        blink_rate = 1000;
        break;

    }
  }
}

/**
   helper functions
*/

void blink_led() {
  uint32_t cur = millis();
  if (cur - blink_last > blink_rate) {
    blink_last += blink_rate;
    ledToggle();
  }
}

void ledToggle() {
  digitalWrite(PIN_LED, !digitalRead(PIN_LED));
}

void ledOn() {
  digitalWrite(PIN_LED, HIGH);
}

void ledOff() {
  digitalWrite(PIN_LED, LOW);
}

