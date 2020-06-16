#include "Arduino.h"
#include "bbadc.h"

BBADC::BBADC() {
  reset();
}

void BBADC::reset() {
  _adc_sum = 0;
  _adc_max = 0;
  _adc_min = _adc_limit;
  _adc_samples = 0;
}

void BBADC::addSample(uint32_t adc_value) {
  // technically we can overflow the sum after ~1048832 samples of 4095
  // so if we reach 1000000 samples without being reset it's time to reset ourselves

  if (_adc_samples > 1000000) {
    reset();
  }
  _adc_samples++;
  _adc_sum += adc_value;

  if (adc_value > _adc_max) _adc_max = adc_value;
  if (adc_value < _adc_min) _adc_min = adc_value;
}

uint32_t BBADC::getMin() {
  return _adc_min;
}

uint32_t BBADC::getMax() {
  return _adc_max;
}

uint32_t BBADC::getSamples() {
  return _adc_samples;
}

uint32_t BBADC::getSum() {
  return _adc_sum;
}

uint32_t BBADC::getAverage() {
  if(_adc_samples > 0) {
    return _adc_sum / _adc_samples;
  }
  return 0;
}
