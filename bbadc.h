#ifndef BBADC_H
#define BBADC_H

#include "Arduino.h"

class BBADC
{
  public:
    BBADC();
    void addSample(uint32_t adc_value);
    void reset();

    uint32_t getMin();
    uint32_t getMax();
    uint32_t getSamples();
    uint32_t getSum();
    uint32_t getAverage();

  private:
    const uint32_t _adc_limit = 4095;
    uint32_t _adc_sum;
    uint32_t _adc_max;
    uint32_t _adc_min;
    uint32_t _adc_samples;
};

#endif
