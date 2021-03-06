#include "pwm.h"
#include "ch.h"
#include "hal.h"

#include <initializer_list>
#include <utility>
#include <cmath>

void initPwm() {
  static const PWMConfig pwmcfg = {
    50000000,  // PWM clock frequency.
    MAX_BRIGHTNESS,     // PWM period.
    NULL,
    {
     {PWM_OUTPUT_ACTIVE_HIGH, NULL},
     {PWM_OUTPUT_ACTIVE_HIGH, NULL},
     {PWM_OUTPUT_ACTIVE_HIGH, NULL},
     {PWM_OUTPUT_ACTIVE_HIGH, NULL}
    },
    /* HW dependent part.*/
    0,
    0
  };
  pwmStart(&PWMD3, &pwmcfg);
  pwmStart(&PWMD4, &pwmcfg);

  for (auto&[port, pin] : std::initializer_list<std::pair<GPIO_TypeDef*, int>>{
      {GPIOA, 6}, {GPIOA, 7}, {GPIOB, 0}, {GPIOB, 1}, // TIM3
      {GPIOB, 6}, {GPIOB, 7}, {GPIOB, 8}, {GPIOB, 9}  // TIM4
      }) {
    palSetPadMode(port, pin, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  }

  for (int channel = 0; channel < 8; ++channel) {
    setPwm(channel, 0);
  }
}

void setPwm(int channel, int brightness) {
  int duty_cycle = 0;
  if (brightness > 0) {
    // Apply logariphmic dimming curve. Parameters adoped from DALI specification.
    float dali_brightness = std::pow(10.f, (float(brightness - 1) * 3 / (MAX_BRIGHTNESS - 1))) - 1;
    constexpr int MIN_VISIBLE_BRIGHTNESS = 2;
    duty_cycle = MIN_VISIBLE_BRIGHTNESS + (MAX_BRIGHTNESS - MIN_VISIBLE_BRIGHTNESS) * 0.5 * dali_brightness / 1000;
  }

  if (channel <=3) {
    pwmEnableChannel(&PWMD3, channel, duty_cycle);
  } else {
    pwmEnableChannel(&PWMD4, channel - 4, duty_cycle);
  }
}
