#include "lights_controller.h"

#include <algorithm>
#include <cmath>

#include "hal.h"

#include "hal/ws2812/ws2812.h"
#include "hsv2rgb.h"

namespace {
constexpr double PI = std::acos(-1);
// Number of RGB leds in a ring.
constexpr int RGB_LEDS = 53;
// Good value for tree lamp.
#if defined LIGHT_HARDWARE_TREE
constexpr float GLIMMER_SPEED = 0.00001;
#endif // LIGHT_HARDWARE_TREE
#if defined LIGHT_HARDWARE_TABLE
constexpr float GLIMMER_SPEED = 0.0005;
#endif // LIGHT_HARDWARE_TABLE

constexpr uint32_t INDICATOR_BAR_BASE = RGB_LEDS * 2 - 4;

THD_WORKING_AREA(glimmerThreadArea, 256);
__attribute__((noreturn)) THD_FUNCTION(glimmerThread, controller) {
  reinterpret_cast<LightsController*>(controller)->run_effects();
}
} // namespace

void LightsController::write_led(uint32_t led_number, uint8_t r, uint8_t g, uint8_t b) {
  if (led_number >= INDICATOR_BAR_BASE - indicator_bar_size_ && led_number < INDICATOR_BAR_BASE) {
    if (led_number >= INDICATOR_BAR_BASE + indicator_bar_min_value_ - indicator_bar_size_ &&
        led_number < INDICATOR_BAR_BASE + indicator_bar_max_value_ - indicator_bar_size_) {
      ws2812_write_led(led_number, 0, 255, 0);
    } else {
      ws2812_write_led(led_number, 0, 0, 0);
    }
  } else {
    ws2812_write_led(led_number, r, g, b);
  }
}

void LightsController::refresh_ring(int base_channel, int direction) {
  if (!rgb_on_) {
    if (rgb_on_previous_) {
      for (int channel = 0; channel < RGB_LEDS * 2; ++channel) {
        write_led(base_channel + channel * direction, 0, 0, 0);
      }
    }
    rgb_on_previous_ = rgb_on_;
    return;
  }
  rgb_on_previous_ = rgb_on_;

  const float warm_hue = 35;
  if (effect_ == Effect::WARM) {
    for (int channel = 0; channel < RGB_LEDS; ++channel) {
      float saturation = 0.5 + std::abs(std::fmod(glimmer_position_ * 0.05 + channel * 2.0f / (RGB_LEDS + 1), 1) - 0.5);
      auto rgb = HSVtoRGB(warm_hue, saturation, rgb_brightness_);
      write_led(base_channel + channel * direction, rgb.r, rgb.g, rgb.b);
    }
    return;
  }
  if (effect_ == Effect::STEADY_PRESET) {
    for (int channel = 0; channel < RGB_LEDS; ++channel) {
      const std::pair<float, float> presets[] = {
          {0, 0.8},    // Pink.
          {20, 0.9},   // Warm white.
          {20, 0.7},   // Warm white.
          {20, 0.0},   // White.
          {225, 0.95}, // Cool blue
          {270, 0.9},  // Purple.
          {270, 1.0}   // Deep purple.
      };
      const auto& preset = presets[int(glimmer_position_ / 10) % std::distance(std::end(presets), std::begin(presets))];
      auto rgb = HSVtoRGB(preset.first, preset.second, rgb_brightness_);
      write_led(base_channel + channel * direction, rgb.r, rgb.g, rgb.b);
    }
    return;
  }

  bool rainbow_on = effect_ == Effect::RAINBOW;
  float speed = rainbow_on ? 40 : 10;
  for (int channel = 0; channel < RGB_LEDS; ++channel) {
    // Standard HSV circle has more cold colors. To make lamp more cosy and warm we expand first 90 degrees (red to yellow) 3x.
    float hue = std::fmod(glimmer_position_ * speed + channel * 540.f / (RGB_LEDS + 1) * rainbow_on, 540);
    if (hue < 270) {
      hue /= 3;
    } else {
      hue -= 180;
    }
    
    auto rgb = HSVtoRGB(hue, 1, rgb_brightness_);
    write_led(base_channel + channel * direction, rgb.r, rgb.g, rgb.b);
  }
}

void LightsController::refresh() {
  static constexpr int MIN_MARGIN = 10;
  static constexpr int MAX_MARGIN = 10000;
  const int glimmer_range = std::max(0, std::min(brightness_ - MIN_MARGIN, MAX_BRIGHTNESS - MAX_MARGIN - brightness_));
  for (int channel = 0; channel < 8; ++channel) {
    int glimmer_delta = 0;
    if (glimmer_speed_ > 0) {
      glimmer_delta = glimmer_range * std::sin(glimmer_position_ + 2 * PI / 8 * GLIMMER_CHANNEL_MAP[channel]);
    }

    setPwm(channel, is_on_ ? brightness_ + glimmer_delta : 0);
  }

  // Tree lamp effect.
  #if defined LIGHT_HARDWARE_TREE
  refresh_ring(6, 1);
  #endif // LIGHT_HARDWARE_TREE

  // Cofee table effect.
  #if defined LIGHT_HARDWARE_TABLE
  refresh_ring(0, 1);
  refresh_ring(53, 1);
  #endif // LIGHT_HARDWARE_TABLE
}

__attribute__((noreturn)) void LightsController::run_effects() {
  systime_t lastCycleTime = chVTGetSystemTimeX();
  for (;;) {
    systime_t cycleTime = chVTGetSystemTimeX();
    glimmer_position_ += TIME_I2MS(chTimeDiffX(lastCycleTime, cycleTime)) * GLIMMER_SPEED * std::pow(2, glimmer_speed_);
    lastCycleTime = cycleTime;

    refresh();

    chThdSleepMilliseconds(2);
  }
}

LightsController* lights_controller() {
  static LightsController lights_controller;
  return &lights_controller;
}

void init_light_controller() {
  chThdCreateStatic(glimmerThreadArea, sizeof(glimmerThreadArea), NORMALPRIO, glimmerThread, lights_controller());
}
