#include <algorithm>
#include <cmath>

#include "ch.h"
#include "hal.h"

#include "shell.h"

#include "hal/blinker.h"
#include "hal/button_handler.h"
#include "hal/pwm.h"
#include "hal/touch.h"
#include "hal/ws2812/ws2812.h"
#include "usb/usbcfg.h"

#include "lib/commands.h"
#include "lib/lights_controller.h"

#include "chprintf.h"

namespace {

constexpr ADCConversionGroup zigbee_brightness{
    .circular=false,
    .num_channels=1,
    .end_cb=NULL,
    .error_cb=NULL,
    0, 0,                         /* CR1, CR2 */
    ADC_SMPR2_SMP_AN0(ADC_SAMPLE_41P5),
    0,                            /* SMPR2 */
    ADC_SQR1_NUM_CH(1),
    0,                            /* SQR2 */
    ADC_SQR3_SQ1_N(5)
};

constexpr ADCConversionGroup zigbee_color{
    .circular=false,
    .num_channels=1,
    .end_cb=NULL,
    .error_cb=NULL,
    0, 0,                         /* CR1, CR2 */
    ADC_SMPR2_SMP_AN0(ADC_SAMPLE_41P5),
    0,                            /* SMPR2 */
    ADC_SQR1_NUM_CH(1),
    0,                            /* SQR2 */
    ADC_SQR3_SQ1_N(4)
};

class SpikeFilter {
 public:
  void add(adcsample_t value) {
    buf_[pos_] = value;
    pos_ = (pos_ + 1) % size;
    value_ = (value_ * (smooth_factor - 1) + averaged_value()) / smooth_factor;
  }

  float value() const { return value_; }

 private:
  static constexpr size_t size = 9;
  static constexpr float smooth_factor = 10;
  adcsample_t buf_[size] = {};
  int pos_ = 0;
  float value_ = 0;

  adcsample_t averaged_value() const {
    adcsample_t buf[size];
    std::copy(std::begin(buf_), std::end(buf_), std::begin(buf));
    std::sort(std::begin(buf), std::end(buf));
    return (buf[size/2] + buf[size/2 + 1] + buf[size/2 + 2]) / 3;
  }
};

THD_WORKING_AREA(zigBeeThreadArea, 512);
__attribute__((noreturn)) THD_FUNCTION(zigBeeThread, arg) {
  (void)arg;
  chRegSetThreadName("ZigBee");

  bool in_control = false;

  SpikeFilter smooth_brightness;
  SpikeFilter smooth_color;

  palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
  for(;;) {
    adcsample_t brightness_raw, color_raw;
    chMtxLock(&adc_mutex);
    adcConvert(&ADCD1, &zigbee_brightness, &brightness_raw, 1);
    adcConvert(&ADCD1, &zigbee_color, &color_raw, 1);
    chMtxUnlock(&adc_mutex);

    smooth_brightness.add(brightness_raw);
    smooth_color.add(color_raw);

    float brightness = std::max(0.f, float(smooth_brightness.value() - 15)) / 3900.f;

    #if defined LIGHT_HARDWARE_TREE
    // TRÅDFRI applies dimming curve itself. As we have our own curve, here we negate it.
    brightness = std::min(1.0, log10(brightness * 100 + 1) / 2);
    if (brightness > 0) {
      lights_controller()->set_brightness(int(brightness * MAX_BRIGHTNESS));
      lights_controller()->set_pixie_on(true);
      in_control = true;
    } else if (in_control) {
      lights_controller()->set_pixie_on(false);
      // In case if the lamp is switched on manually restore at a reasonable brightness.
      lights_controller()->set_brightness(MAX_BRIGHTNESS);
      in_control = false;
    }

    if (in_control) {
      static constexpr float MAX_GLIMMER_SPEED = 9.;
      lights_controller()->set_glimmer_speed(std::max(0, int(smooth_color.value()) - 20) * MAX_GLIMMER_SPEED / 3900.);
    }
    #endif // LIGHT_HARDWARE_TREE

    #if defined LIGHT_HARDWARE_TABLE
    if (brightness > 0) {
      lights_controller()->set_rgb_brightness(brightness);
      lights_controller()->set_rgb_on(true);
      in_control = true;
    } else if (in_control) {
      lights_controller()->set_rgb_on(false);
      // In case if the lamp is switched on manually restore at a reasonable brightness.
      lights_controller()->set_rgb_brightness(0.5);
      in_control = false;
    }

    if (in_control) {
        float normalized_color = std::max(0.f, std::min(1.f, (smooth_color.value() - 20) / 3900.f));
        // Don't want MAX_EFFECT to be selected.
        normalized_color -= 0.01;
        lights_controller()->set_effect(
            static_cast<Effect>(int(normalized_color * int(Effect::MAX_EFFECT))));
    }
    #endif // LIGHT_HARDWARE_TABLE

    chThdSleepMilliseconds(25);
  }
}

void zigbee_state(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void) argc;
  (void) argv;
  // A helper utility that prints zigbee state.
  for(;;) {
    adcsample_t brightness, color;
    adcConvert(&ADCD1, &zigbee_brightness, &brightness, 1);
    adcConvert(&ADCD1, &zigbee_color, &color, 1);
    chprintf(chp, "Brightness: %d\r\n", int(brightness));
    chprintf(chp, "Color: %d\r\n", int(color));
    chThdSleepMilliseconds(250);
  }
}

#if defined LIGHT_HARDWARE_TREE

THD_WORKING_AREA(button1HandlerThreadArea, 256);
__attribute__((noreturn)) THD_FUNCTION(button1HandlerThread, arg) {
  (void)arg;
  chRegSetThreadName("on_button_handler");

  class Handler : public ButtonHandler {
  public:
    void onClick() override {
      lights_controller()->set_pixie_on(!lights_controller()->pixie_on());
    }

    void onLongPress() override {
      lights_controller()->set_rgb_on(!lights_controller()->rgb_on());
    }
  } handler;
  ButtonController controller(GPIOA, 0, TIME_MS2I(LONG_PRESS_TIME_MS), &handler);
  controller();
}

THD_WORKING_AREA(button2HandlerThreadArea, 256);
__attribute__((noreturn)) THD_FUNCTION(button2HandlerThread, arg) {
  (void)arg;
  chRegSetThreadName("brightness_button_handler");

  class Handler : public ButtonHandler {
  public:
    void onClick() override {
      lights_controller()->set_glimmer_speed(lights_controller()->glimmer_speed() > 0 ? 0 : 4);
    }

    void onKeyDown() override {
      first_iteration_ = true;
    }

    void onKeyHeldDown() override {
      if (first_iteration_) {
        first_iteration_ = false;
        fade_start_ = chVTGetSystemTimeX() - 1;
      }

      int elapsed = chVTTimeElapsedSinceX(fade_start_);
      fade_start_ = chVTGetSystemTimeX();
      idle_time_ = std::max(0, idle_time_ - elapsed);
      if (idle_time_ > 0) {
        return;
      }

      int brightness = lights_controller()->brightness() + brightness_inc_ * elapsed;
      if (brightness < MIN_BRIGHTNESS) {
        brightness = MIN_BRIGHTNESS;
        brightness_inc_ = -brightness_inc_;
      }
      if (brightness > MAX_BRIGHTNESS) {
        brightness = MAX_BRIGHTNESS;
        brightness_inc_ = -brightness_inc_;
      }
      lights_controller()->set_brightness(brightness);

      if (brightness == MAX_BRIGHTNESS || brightness == MIN_BRIGHTNESS) {
        idle_time_ = TIME_MS2I(1000);
      }
    }

  private:
    int brightness_inc_ = -10;
    bool first_iteration_;
    systime_t fade_start_;
    int idle_time_ = 0;
  } handler;
  ButtonController controller(GPIOA, 1, TIME_MS2I(1), &handler);
  controller();
}

#endif // LIGHT_HARDWARE_TREE

#if defined LIGHT_HARDWARE_TABLE

int raw_touch_values[4] = {};
int touch_values[4] = {};

void touch_calibration(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void) argc;
  (void) argv;
  // A helper utility that prints touch sensor values to adjust sensetivity.
  for(;;) {
    chprintf(chp, "R %d\t%d\t%d\t%d\r\n", raw_touch_values[0], raw_touch_values[1], raw_touch_values[2], raw_touch_values[3]);
    chprintf(chp, "  %d\t%d\t%d\t%d\r\n", touch_values[0], touch_values[1], touch_values[2], touch_values[3]);
    chThdSleepMilliseconds(500);
  }
}

enum Buttons {
  ON_OF_BUTTON = 0,
  BRIGHTNESS_BUTTON = 2,
  GLIMMER_BUTTON = 1,
  EFFECT_BUTTON = 3
};

const int BUTTON_LED[] = {
  /*ON_OF_BUTTON=*/106,
  /*BRIGHTNESS_BUTTON=*/108,
  /*GLIMMER_BUTTON=*/107,
  /*EFFECT_BUTTON=*/109
};

THD_WORKING_AREA(touchThreadArea, 256);
__attribute__((noreturn)) THD_FUNCTION(touchThread, arg) {
  (void)arg;
  chRegSetThreadName("touch");

  const int min_value = 35;
  const int threshold = 30;
  const int max_value = 100;

  const int press_period = 2;
  const int hold_period = 5;

  bool is_locked = false;

  // Non ascending order is intentional.
  ADCTouchSensor sensors[] = {{0}, {1}, {2}, {3}};
  int pressed[4] = {};
  for (;;) {
    for (int i = 0; i < 4; ++i) {
      std::tie(raw_touch_values[i], touch_values[i]) = sensors[i].read_both();

      int value = touch_values[i];

      if (value > threshold) {
        ++pressed[i];
      } else {
        pressed[i] = 0;
      }

      if (!is_locked) {
        int intensity = 128 * (value > min_value) + 128 * (value > threshold);
        float tone = float(max_value) / std::min(value, max_value);
        ws2812_write_led(BUTTON_LED[i], int((1 - tone) * intensity), int(tone * intensity), 0);
      }
    }

    if (pressed[ON_OF_BUTTON] && pressed[EFFECT_BUTTON]) {
      if (pressed[ON_OF_BUTTON] == hold_period) {
        is_locked = !is_locked;
      }
      if (is_locked) {
        ws2812_write_led(BUTTON_LED[0], 255, 0, 0);
        ws2812_write_led(BUTTON_LED[1], 255, 0, 0);
        ws2812_write_led(BUTTON_LED[2], 255, 0, 0);
        ws2812_write_led(BUTTON_LED[3], 255, 0, 0);
      }
      continue;
    }

    if (pressed[0] == 0 && pressed[1] == 0 && pressed[2] == 0 && pressed[3] == 0) {
      lights_controller()->set_indicator_bar_min_value(0);
      lights_controller()->set_indicator_bar_size(0);
    }

    if (!is_locked) {
      if (pressed[ON_OF_BUTTON] == press_period) {
        lights_controller()->set_rgb_on(!lights_controller()->rgb_on());
      }
      if (pressed[BRIGHTNESS_BUTTON] == press_period) {
        float brightness = lights_controller()->rgb_brightness();
        brightness = 0.125 + std::fmod(brightness, 1.0);
        lights_controller()->set_rgb_brightness(brightness);

        lights_controller()->set_indicator_bar_size(7);
        lights_controller()->set_indicator_bar_max_value(int(brightness * 8 - 1));

        // Reset the counter to achieve looping.
        pressed[BRIGHTNESS_BUTTON] = 0;
      }
      if (pressed[GLIMMER_BUTTON] == press_period) {
        lights_controller()->set_glimmer_speed(int(lights_controller()->glimmer_speed() + 1) % 4);

        lights_controller()->set_indicator_bar_size(3);
        lights_controller()->set_indicator_bar_max_value(int(lights_controller()->glimmer_speed()));

        // Reset the counter to achieve looping.
        pressed[GLIMMER_BUTTON] = 0;
      }
      if (pressed[EFFECT_BUTTON] == press_period) {
        const int effect_index = static_cast<int>(lights_controller()->effect());
        const int effects_count =  static_cast<int>(Effect::MAX_EFFECT);

        lights_controller()->set_effect(
            static_cast<Effect>((effect_index + 1) % effects_count));
        if (lights_controller()->rgb_on()) {
          lights_controller()->set_rgb_on(false);
          chThdSleepMilliseconds(100);
          lights_controller()->set_rgb_on(true);
        }
        lights_controller()->set_indicator_bar_size(effects_count);
        lights_controller()->set_indicator_bar_min_value(effect_index);
        lights_controller()->set_indicator_bar_max_value(effect_index + 1);

        // Reset the counter to achieve looping.
        pressed[EFFECT_BUTTON] = 0;
      }
    } else {
      if (pressed[0] > 1 || pressed[1] > 1 || pressed[2] > 1 || pressed[3] > 1) {
        ws2812_write_led(BUTTON_LED[0], 255, 0, 0);
        ws2812_write_led(BUTTON_LED[1], 255, 0, 0);
        ws2812_write_led(BUTTON_LED[2], 255, 0, 0);
        ws2812_write_led(BUTTON_LED[3], 255, 0, 0);
      } else {
        ws2812_write_led(BUTTON_LED[0], 0, 0, 0);
        ws2812_write_led(BUTTON_LED[1], 0, 0, 0);
        ws2812_write_led(BUTTON_LED[2], 0, 0, 0);
        ws2812_write_led(BUTTON_LED[3], 0, 0, 0);
      }
    }
  }
}

#endif // LIGHT_HARDWARE_TABLE

// USB shell.
#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

const ShellCommand commands[] = {
  // Low level LED controls.
  {"pwm", commands::pwm},
  {"rgb", commands::rgb},
  {"hsv", commands::hsv},
  // Light asm interpreter.
  {"play", commands::play},
  // High level LED controls.
  {"brightness", commands::brightness},
  {"pixie_lights", commands::pixie_lights},
  {"rgb_backlight", commands::rgb_backlight},
  {"glimmer", commands::glimmer},
  {"zigbee_state", zigbee_state},
  #if defined LIGHT_HARDWARE_TABLE
    {"touch_calibration", touch_calibration},
  #endif // LIGHT_HARDWARE_TABLE
  {NULL, NULL}
};

const ShellConfig shell_config = {
  (BaseSequentialStream*) &SDU1,
  commands
};

} // namespace


int main(void) {
  chMtxObjectInit(&adc_mutex);
  halInit();
  chSysInit();
  ws2812_init();
  adcStart(&ADCD1, NULL);

  initPwm();
  init_light_controller();

  chThdCreateStatic(zigBeeThreadArea, sizeof(zigBeeThreadArea), NORMALPRIO, zigBeeThread, NULL);

  #if defined LIGHT_HARDWARE_TREE
  // Tree lamp has hardware buttons and EXTI is used to capture them.
  chThdCreateStatic(button1HandlerThreadArea, sizeof(button1HandlerThreadArea), NORMALPRIO, button1HandlerThread, NULL);
  chThdCreateStatic(button2HandlerThreadArea, sizeof(button2HandlerThreadArea), NORMALPRIO, button2HandlerThread, NULL);
  #endif // LIGHT_HARDWARE_TREE

  #if defined LIGHT_HARDWARE_TABLE
  // Cofee table uses touch buttons.
  chThdCreateStatic(touchThreadArea, sizeof(touchThreadArea), NORMALPRIO, touchThread, NULL);
  #endif // LIGHT_HARDWARE_TABLE

  // Initializes a serial-over-USB CDC driver.
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  // Activates the USB driver and then the USB bus pull-up on D+.
  // Note, a delay is inserted in order to not have to disconnect the cable
  // after a reset.
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  shellInit();
  startBlinker();

  // Wait for USB connection and spawn shell.
  for(;;) {
    if (SDU1.config->usbp->state == USB_ACTIVE) {
      thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                              "shell", NORMALPRIO + 1,
                                              shellThread, (void *)&shell_config);
      chThdWait(shelltp);
      chThdRelease(shelltp);
    }
    chThdSleepMilliseconds(1000);
  }
}
