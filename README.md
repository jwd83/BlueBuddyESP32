# BlueBuddyESP32

A simple ESP32 based USB to Bluetooth bridge and basic platform

## bluebuddy command mode commands/responses
### single byte commands

    a     - a:xxx,...     - return latest value ADC value
    A     - A:xxx,...     - return average of ring buffered ADC value
    b     - [none]        - change to bridge mode (usb serial <-> bluetooth serial)
    d     - d:xxx         - return digital inputs
    h     - h:xxx         - return free heap space
    p     - p:xxx,...     - return analog data from bbADC (summed average, min, max, sample count)
    q     - [none]        - quit (disconnect and wait for new pairing)
    r     - r:r1,xxx,etc  - return stored resistor network
    s     - s:xxx         - return total analog samples
    v     - v:xxx,...     - return latest analog voltage (based on resistor network)
    V     - V:xxx,...     - return average of ring buffered analog voltage (based on resistor network)

### multibyte commands (prefaced with $)

    $                     - prepare for a multibyte command....
    $1xxx - DAC:1,xxx...  - set DAC1 to value xxx (0-255)
    $2xxx - DAC:2,xxx...  - set DAC2 to value xxx (0-255)

## Todo

1. Add bridge mode to command mode escape sequence

