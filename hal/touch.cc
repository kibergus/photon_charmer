#include "touch.h"

#include <algorithm>
#include <limits>

mutex_t adc_mutex;

namespace {

const int max_calibration_samples = 1000;

constexpr ADCConversionGroup ground_channel = {
  .circular=false,
  .num_channels=1,
  .end_cb=NULL,
  .error_cb=NULL,
  0, 0,                         /* CR1, CR2 */
  ADC_SMPR1_SMP_AN15(ADC_SAMPLE_7P5),
  0,                            /* SMPR2 */
  ADC_SQR1_NUM_CH(1),
  0,                            /* SQR2 */
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN15)
};

} // namespace

ADCTouchSensor::ADCTouchSensor(iopadid_t pad)
  // Port is non configurable as ADC channels depend on port selection.
  : port_(GPIOA)
  , pad_(pad)
  , measurement_channel_{
      .circular=false,
      .num_channels=1,
      .end_cb=NULL,
      .error_cb=NULL,
      0, 0,                         /* CR1, CR2 */
      ADC_SMPR2_SMP_AN0(ADC_SAMPLE_7P5),
      0,                            /* SMPR2 */
      ADC_SQR1_NUM_CH(1),
      0,                            /* SQR2 */
      ADC_SQR3_SQ1_N(pad)}
  , reference_(std::numeric_limits<int>::max())
  , calibration_buffer_(0)
  , calibration_samples_(0)
  , max_calibration_samples_(100) {
}

void ADCTouchSensor::ground() {
  adcsample_t sample;
  adcConvert(&ADCD1, &ground_channel, &sample, 1);
}

int ADCTouchSensor::read(unsigned samples) {
  return std::max(0, read_raw(samples) - reference_);
}

std::pair<int, int> ADCTouchSensor::read_both(unsigned samples) {
  int value = read_raw(samples);
  return {value, std::max(0, read_raw(samples) - reference_)};
}

int ADCTouchSensor::read_raw(unsigned samples) {
  int32_t total = 0;
  for (unsigned i = 0; i < samples; i++) {
    palSetPad(port_, pad_);
    palSetPadMode(port_, pad_, PAL_MODE_OUTPUT_PUSHPULL);
    chThdSleep(TIME_US2I(30));

    chMtxLock(&adc_mutex);
    ground();
    palSetPadMode(port_, pad_, PAL_MODE_INPUT_ANALOG);
    adcsample_t sample;
    adcConvert(&ADCD1, &measurement_channel_, &sample, 1);
    chMtxUnlock(&adc_mutex);

    total += sample;
  }

  palClearPad(port_, pad_);
  palSetPadMode(port_, pad_, PAL_MODE_OUTPUT_PUSHPULL);

  static const int ADCTOUCH_DIVIDER = 4;
  int result = total / ADCTOUCH_DIVIDER / samples;

  calibration_buffer_ += result;
  ++calibration_samples_;
  if (calibration_samples_ == max_calibration_samples_) {
    reference_ = calibration_buffer_ / calibration_samples_;
    max_calibration_samples_ = std::min(max_calibration_samples_ * 2, max_calibration_samples);
  }

  return result;
}
