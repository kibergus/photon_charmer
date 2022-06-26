#pragma once

#include <utility>
#include "hal.h"

// This mutex must be locked each time ADC is used.
extern mutex_t adc_mutex;

// Touch capacitance reader. No external circuosity needed, internal pullup
// resistors and ADC capacitor are used.
class ADCTouchSensor
{
 public:
  // Reads capacitance of the given pad in port GPIOA.
  ADCTouchSensor(iopadid_t pad);

  int read(unsigned samples = 15);

  // Returns raw and calibrated values.
  std::pair<int, int> read_both(unsigned samples = 15);

  // Read value not compensated for pad capacitance;
  int read_raw(unsigned samples = 15);

 private:
  // Discharges ADC capacitor to the ground pin.
  void ground();

  ioportid_t port_;
  iopadid_t pad_;
  ADCConversionGroup measurement_channel_;

  int reference_;

  int calibration_buffer_;
  int calibration_samples_;
  int max_calibration_samples_;
};
