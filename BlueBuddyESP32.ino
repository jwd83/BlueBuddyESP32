/*
   bluebuddy accessory
   reformat code is Ctrl+T

   add bridge mode to command mode escape (maybe 32c in a row?)

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


#define PIN_RXD                       1
#define PIN_DIGITAL_IN_1              2
#define PIN_TXD                       3
#define PIN_CAN_RX                    4
#define PIN_CAN_TX                    5     /* are CAN TX and the LED shared on iot bus? */
#define PIN_LED                       5
// #define PIN_SPI_FLASH_1            6     /* reserved for the ESP32's SPI flash */
// #define PIN_SPI_FLASH_2            7     /* reserved for the ESP32's SPI flash */
// #define PIN_SPI_FLASH_3            8     /* reserved for the ESP32's SPI flash */
// #define PIN_SPI_FLASH_4            9     /* reserved for the ESP32's SPI flash */
// #define PIN_SPI_FLASH_5            10    /* reserved for the ESP32's SPI flash */
// #define PIN_SPI_FLASH_6            11    /* reserved for the ESP32's SPI flash */
#define PIN_DIGITAL_IN_2              16
#define PIN_DIGITAL_IN_3              17
#define PIN_SDA                       21
#define PIN_SCL                       22
#define PIN_DAC1                      25
#define PIN_DAC2                      26


#define PIN_COUNT_ADC                 6
#define PIN_COUNT_DIGITAL_INPUTS      3

#define SERIAL_BAUD           115200

#define MODE_BRIDGE           1
#define MODE_COMMAND          2

#define ANALOG_SAMPLE_MS      50
#define ADC_MAX_BITS          4095.0
#define ADC_MAX_VOLT          3.3

/*
   globals
*/
// const uint32_t adc_pins[PIN_COUNT_ADC] = {36, 39, 32, 33, 34, 35};
const uint32_t adc_pins[PIN_COUNT_ADC] = {0, 3, 4, 5, 6, 7};
const double r1[PIN_COUNT_ADC] = {500000,  500000,  100000, 100000,   0, 0 };
const double r2[PIN_COUNT_ADC] = {100000,  100000,  100000, 100000,   0, 0 };
const uint32_t digital_input_pins[PIN_COUNT_DIGITAL_INPUTS] = {
  PIN_DIGITAL_IN_1, 
  PIN_DIGITAL_IN_2, 
  PIN_DIGITAL_IN_3
};

BluetoothSerial SerialBT; //Object for Bluetooth

uint32_t loop_mode = MODE_COMMAND;
uint32_t blink_last = 0;
uint32_t blink_rate = 1000;
uint32_t adc_values[PIN_COUNT_ADC] = {0, 0, 0, 0, 0, 0};

/*
   SETUP
   turn on bluetooth, wait for connection?
*/

void setup() {

  // setup our pins

  // setup our digital inputs
  for(int i = 0; i < PIN_COUNT_DIGITAL_INPUTS; i++) {
    pinMode(digital_input_pins[i], INPUT);
    
  }
  pinMode(PIN_LED, OUTPUT);


  // turn LED solid when booting
  ledOn();

  // turn on serial
  Serial.begin(115200);
  Serial.println("Serial....started");
  Serial.print("Bluetooth...");

  // start pairing mode
  SerialBT.begin("BlueBuddy");
  Serial.println("started");
}

/*
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
      sample_analogs();
      respond_to_command();
      break;

  }
}

void sample_analogs() {
  for (int i = 0; i < PIN_COUNT_ADC; i++) {
    adc_values[i] = analogRead(adc_pins[i]);
  }
}


void respond_to_command() {
  char in = 0;

  if (SerialBT.available()) {
    in = SerialBT.read();
    switch (in) {
      /*
         0 turns the LED off
      */
      case '0':
        ledOff();
        break;


      /*
         1 turns the LED on
      */
      case '1':
        ledOn();
        break;

      /*
         instant sample of Analog inputs
      */
      case 'a':

        for (int i = 0; i < PIN_COUNT_ADC; i++) {
          SerialBT.print(adc_values[i]);
          if (i < PIN_COUNT_ADC - 1) {
            SerialBT.print(",");
          }
        }
        SerialBT.println();
        break;

      /*
         change to bridge mode
      */
      case 'b':
        loop_mode = MODE_BRIDGE;
        break;

      /*
         get digital states
      */
      case 'd':
        for (int i = 0; i < PIN_COUNT_DIGITAL_INPUTS; i++) {
          if (digitalRead(digital_input_pins[i]) == HIGH) {
            SerialBT.print("1");
          } else {
            SerialBT.print("0");
          }
        }
        SerialBT.println();
        break;

      /*
         q quit/disconnect
      */
      case 'q':
        // disconnect
        SerialBT.end();

        // todo investigate adding delay before restarting pairing. device may just reconnect.
        // using a bluetooth terminal this seems to work fine without delay.

        // begin pairing mode for next device
        SerialBT.begin("BlueBuddy");
        break;

      /*
         report free Heap memory
      */
      case 'h':
        SerialBT.println(ESP.getFreeHeap());
        break;


      /*
         report Resistor divider network
      */
      case 'r':

        SerialBT.print("r1,");
        for (int i = 0; i < PIN_COUNT_ADC; i++) {
          SerialBT.print(r1[i], 0);
          SerialBT.print(",");
        }
        SerialBT.print("r2,");
        for (int i = 0; i < PIN_COUNT_ADC; i++) {
          SerialBT.print(r2[i], 0);
          if (i < PIN_COUNT_ADC - 1) {
            SerialBT.print(",");
          }
        }
        SerialBT.println();
        break;


      /*
         instant sample of Analog voltages based on divider network
      */
      case 'v':

        /*
           loop through our analog pins
        */
        for (int i = 0; i < PIN_COUNT_ADC; i++) {

          /*
             store our adc reading and solve the pin voltage
          */
          double v = (double)adc_values[i];
          v = v / ADC_MAX_BITS * ADC_MAX_VOLT;


          /*
             check if there is a resistor network in place to solve the input signal voltage
          */


          if (r1[i] != 0 && r2[i] != 0) {
            /*
               there is a resistor network, solve original input voltage
            */

            v = (v * (r1[i] + r2[i])) / r2[i];
          }
          SerialBT.print(v, 3);
          if (i < PIN_COUNT_ADC - 1) {
            SerialBT.print(",");
          }
        }
        SerialBT.println();
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
