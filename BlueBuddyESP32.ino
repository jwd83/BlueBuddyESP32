/*
  BlueBuddyESP32

    bluebuddy command mode commands/responses

    single byte commands

    a     - a:xxx,...     - return latest value ADC value
    A     - A:xxx,...     - return average of ring buffered ADC value
    b     - [none]        - change to bridge mode (usb serial <-> bluetooth serial)
    d     - d:xxx         - return digital inputs
    h     - h:xxx         - return free heap space
    q     - [none]        - quit (disconnect and wait for new pairing)
    r     - r:r1,xxx,etc  - return stored resistor network
    s     - s:xxx         - return total analog samples
    v     - v:xxx,...     - return latest analog voltage (based on resistor network)
    V     - V:xxx,...     - return average of ring buffered analog voltage (based on resistor network)

    multibyte commands (prefaced with $)
    
    $                     - prepare for a multibyte command....
    $1xxx - DAC:1,xxx...  - set DAC1 to value xxx (0-255)
    $2xxx - DAC:2,xxx...  - set DAC2 to value xxx (0-255)

    todo

    add bridge mode to command mode escape (maybe 32c in a row?)
    add o command to return DAC outputs
    document responses
*/

// include files

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

#define SERIAL_BAUD                   115200

#define MODE_BRIDGE                   1
#define MODE_COMMAND                  2

#define ADC_MAX_BITS                  4095.0
#define ADC_MAX_VOLT                  3.3
#define ADC_RING_SIZE                 32        // num channels * 4 bytes per sample * ring size


// settings to clean up the ESP32 analog inputs that are a bit on the noisy side compared to an atmel

// THESE WORK WELL!
// #define ADC_DISCARD                   20
// #define ADC_OVERSAMPLE                20
// #define ADC_PRE_SAMPLE_DELAY          25
// #define ADC_DELAY_BEFORE_DISCARD      true

// experimental (these also work well)
#define ADC_DISCARD                   20
#define ADC_OVERSAMPLE                10
#define ADC_PRE_SAMPLE_DELAY          0
#define ADC_DELAY_BEFORE_DISCARD      true

/*
   globals
*/

// immutable globals
const uint32_t adc_pins[PIN_COUNT_ADC] = {36, 39, 32, 33, 34, 35};
// const uint32_t adc_pins[PIN_COUNT_ADC] = {0, 3, 4, 5, 6, 7};
// const double r1[PIN_COUNT_ADC] = {500000,  500000,  100000, 100000,   0, 0 };
// const double r2[PIN_COUNT_ADC] = {100000,  100000,  100000, 100000,   0, 0 };
const double r1[PIN_COUNT_ADC] = {0,  0,  0, 0, 0, 0 };
const double r2[PIN_COUNT_ADC] = {0,  0,  0, 0, 0, 0 };

const uint32_t digital_input_pins[PIN_COUNT_DIGITAL_INPUTS] = {
  PIN_DIGITAL_IN_1,
  PIN_DIGITAL_IN_2,
  PIN_DIGITAL_IN_3
};
const uint32_t command_window = 1000;


// mutable globals
BluetoothSerial SerialBT; //Object for Bluetooth

uint32_t loop_mode = MODE_COMMAND;
uint32_t blink_last = 0;
uint32_t blink_rate = 1000;
uint32_t adc_ring_buffer[PIN_COUNT_ADC][ADC_RING_SIZE];
int32_t adc_ring_position = -1; // start at -1 as we will increment this
uint32_t sample_count = 0;

// multibyte commands
bool command_in_progress = false;
char command_buffer[4];
uint32_t command_start_time = 0;
int32_t command_bytes_expected = 0;
int command_position = 0;

/*
   SETUP
   turn on bluetooth, wait for connection?
*/

void setup() {

  // setup our pins

  // setup our digital inputs
  for (int i = 0; i < PIN_COUNT_DIGITAL_INPUTS; i++) {
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

  // clear ring buffer out
  clear_ring_buffer();
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
      process_commands();
      break;

  }
}

void sample_analogs() {
  // increment our position in the ring
  adc_ring_position++;
  if (adc_ring_position >= ADC_RING_SIZE) adc_ring_position = 0;

  // store the latest adc values
  for (int i = 0; i < PIN_COUNT_ADC; i++) {
    adc_ring_buffer[i][adc_ring_position] = oversample_adc(adc_pins[i], false);
  }

  // update our sample count
  sample_count++;
}

uint32_t oversample_adc(int channel, bool delay_before_discard) {
  uint32_t discards = 0;
  uint32_t readings = 0;

  if (delay_before_discard && ADC_PRE_SAMPLE_DELAY > 0) delayMicroseconds(ADC_PRE_SAMPLE_DELAY);

  for (int i = 0; i < ADC_DISCARD; i++) {
    discards += analogRead(channel);
  }

  if (!delay_before_discard && ADC_PRE_SAMPLE_DELAY > 0) delayMicroseconds(ADC_PRE_SAMPLE_DELAY);

  for (int i = 0; i < ADC_OVERSAMPLE; i++) {
    readings += analogRead(channel);

  }

  return readings / ADC_OVERSAMPLE;
}

uint32_t read_analog(uint32_t channel, bool buffered_value) {
  uint32_t adc_bits_sum = 0;

  // make sure we got a channel in the requested range
  if (channel >= PIN_COUNT_ADC) {
    // invalid channel
    return 0;
  } else {
    // a valid channel was specified, check if we need to return the raw or buffered value
    // also verify our ring is of an appropriate size (prevent divide by zero)
    if (buffered_value && ADC_RING_SIZE >= 2) {
      // if the buffered value is requested return the average of the values in the ring buffer
      for (int i = 0; i < ADC_RING_SIZE; i++) {
        adc_bits_sum += adc_ring_buffer[channel][i];
      }

      return adc_bits_sum / ADC_RING_SIZE;
    } else {
      // if the raw unbuffered value is requested simply return the latest sample in the ring buffer
      return adc_ring_buffer[channel][adc_ring_position];
    }
  }
}


void process_commands() {
  char in;

  // // multibyte commands
  // bool command_in_progress = true;
  // char command_buffer[4];
  // uint32_t command_start_time = 0;
  // int32_t command_bytes_expected = 0;
  // int command_position = 0;

  // check if the command has timed out
  if (command_in_progress) {
    if (command_start_time + command_window < millis()) {
      command_in_progress = false;
    }
  }

  if (SerialBT.available()) {
    in = SerialBT.read();
    // if there is no command in progress process input as usual
    if (!command_in_progress) {
      if (in == '$') {
        // prepare to read a multibyte command
        command_in_progress = true;
        command_start_time = millis();
        command_position = 0;
      } else {
        // process single byte responses
        process_command_reply(in);
      }
    } else {

      // there is a multi byte command in progress

      // if we haven't yet received a byte for this command then we now determinte which command is being sent
      if (command_position == 0) {
        // determine which command we are receiving
        switch (in) {

          // dac output command
          case '1':
          case '2':
            command_bytes_expected = 4;
            break;

          // unknown command
          default:
            command_in_progress = false;
            break;
        }
      }

      // don't bother processing data for a command not in progress
      if (command_in_progress) {
        command_buffer[command_position] = in;
        command_position++;
        if (command_position == command_bytes_expected) {
          process_multibyte_command();
          command_in_progress = false;
        }
      }
    } // end multibyte command handler
  }
}

void process_multibyte_command() {
  char mbc_tmp_chars[4];
  int mbc_tmp_int;
  switch (command_buffer[0]) {
    case '1':
    case '2':
      // copy the rest of command into our temp char array
      mbc_tmp_chars[0] = command_buffer[1];
      mbc_tmp_chars[1] = command_buffer[2];
      mbc_tmp_chars[2] = command_buffer[3];
      // null terminate the our temp char array
      mbc_tmp_chars[3] = 0;

      // convert value to int
      mbc_tmp_int = constrain(atoi(mbc_tmp_chars), 0, 255);


      // write value to DAC
      if (command_buffer[0] == '1') dacWrite(PIN_DAC1, mbc_tmp_int);
      else dacWrite(PIN_DAC2, mbc_tmp_int);

      // reply DAC:x,yyy where x = channel and yyy = value
      SerialBT.print("DAC:");
      SerialBT.print(command_buffer[0]);
      SerialBT.print(",");
      SerialBT.println(mbc_tmp_int);
      break;
  }
}

void process_command_reply(char in) {
  bool buffered;

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
       sample Analog input bits
       a = instant
       A = ring buffer average
    */
    case 'a':
    case 'A':
      if (in == 'A') {
        buffered = true;
        SerialBT.print("A:");
      } else {
        buffered = false;
        SerialBT.print("a:");
      }

      for (int i = 0; i < PIN_COUNT_ADC; i++) {
        SerialBT.print(read_analog(i, buffered));
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
      SerialBT.print("d:");
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
       report free Heap memory
    */
    case 'h':
      SerialBT.print("h:");
      SerialBT.println(ESP.getFreeHeap());
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
       report Resistor divider network
    */
    case 'r':
      SerialBT.print("r:");
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
       report the Sample count
    */
    case 's':
      SerialBT.print("s:");
      SerialBT.println(sample_count);
      break;

    /*
       instant sample of Analog voltages based on divider network
    */
    case 'v':
    case 'V':
      if (in == 'V') {
        buffered = true;
        SerialBT.print("V:");
      } else {
        buffered = false;
        SerialBT.print("v:");
      }

      /*
         loop through our analog pins
      */
      for (int i = 0; i < PIN_COUNT_ADC; i++) {

        /*
           store our adc reading and solve the pin voltage
        */
        double v = (double) read_analog(i, buffered);
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

void bridge_serials() {
  char sbt_read;

  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }

  if (SerialBT.available()) {
    sbt_read = SerialBT.read();
    Serial.write(sbt_read);
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

void clear_ring_buffer() {
  for (int i = 0; i < PIN_COUNT_ADC; i++) {
    for (int j = 0; j < ADC_RING_SIZE; j++) {
      adc_ring_buffer[i][j] = 0;
    }
  }
}
