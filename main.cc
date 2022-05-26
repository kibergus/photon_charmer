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
      lights_controller()->set_glimmer_on(!lights_controller()->glimmer_on());
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
        lights_controller()->set_glimmer_speed(lights_controller()->glimmer_speed() + 1);

        lights_controller()->set_indicator_bar_size(3);
        lights_controller()->set_indicator_bar_max_value(lights_controller()->glimmer_speed());

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
  halInit();
  chSysInit();
  ws2812_init();
  adcStart(&ADCD1, NULL);

  initPwm();
  init_light_controller();

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
