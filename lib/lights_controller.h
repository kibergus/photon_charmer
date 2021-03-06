#pragma once

#include <stdint.h>
#include "hal/pwm.h"

enum class Effect {
  STEADY_RAINBOW,
  RAINBOW,
  WARM,
  STEADY_PRESET,
  MAX_EFFECT
};

// Implements high-level visual pixie and RGB lights visial effects.
class LightsController {
 public:
  // Worker process that implements effects. Call from a worker thread.
  __attribute__((noreturn)) void run_effects();

  // Toggle the lights.
  bool pixie_on() const { return is_on_; }
  void set_pixie_on(bool on) {
    is_on_ = on;
    refresh();
  }

  // Sets average pixie lights brightness.
  bool brightness() const { return brightness_; }
  void set_brightness(int brightness) {
    brightness_ = brightness;
    refresh();
  }

  // Toggle RGB backlight.
  bool rgb_on() const { return rgb_on_; }
  void set_rgb_on(bool on) {
    rgb_on_ = on;
    refresh();
  }

  // Toggle the glimmer effect.
  int glimmer_speed() const { return glimmer_speed_; }
  void set_glimmer_speed(int speed) { glimmer_speed_ = speed % 4; }

  // Toggle rainbow effect.
  Effect effect() const { return effect_; }
  void set_effect(Effect effect) { effect_ = effect; }

  // Rgb brightness (V component in HSV).
  float rgb_brightness() const { return rgb_brightness_; }
  void set_rgb_brightness(float rgb_brightness) { rgb_brightness_ = rgb_brightness; }

  void set_indicator_bar_size(int size) {indicator_bar_size_ = size;}
  void set_indicator_bar_min_value(int value) {indicator_bar_min_value_ = value;}
  void set_indicator_bar_max_value(int value) {indicator_bar_max_value_ = value;}


 private:
  friend void init_light_controller();
  // Order is set in such a way, that closely placed LED strips blink in couterphase leading to less brightness variation.
  static constexpr int GLIMMER_CHANNEL_MAP[] = {0, 5, 2, 7, 1, 6, 3, 4};

  // Refreshes brightness of all LEDs.
  void refresh_ring(int base_channel, int direction);
  void refresh();

  void write_led(uint32_t led_number, uint8_t r, uint8_t g, uint8_t b);

  // Pixie lights state.
  bool is_on_ = false;
  int brightness_ = MAX_BRIGHTNESS;
  // RGB tree backlight state.
  bool rgb_on_ = false;
  bool rgb_on_previous_ = false;
  float rgb_brightness_ = 1.0;

  // Number of LEDs in an indicator.
  int indicator_bar_size_ = 0;
  int indicator_bar_min_value_ = 0;
  int indicator_bar_max_value_ = 0;

  // Effects state.
  int glimmer_speed_ = 1.0;
  float glimmer_position_ = 0.0;
  Effect effect_ = Effect::RAINBOW;
};

// Returns a singleton initialized instance of the lights controller.
LightsController* lights_controller();
// Starts a worker thread for the global controller.
void init_light_controller();
