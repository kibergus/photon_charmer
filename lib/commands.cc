#include "commands.h"

#include "hal/pwm.h"
#include "hal/ws2812/ws2812.h"
#include "lib/asm.h"
#include "lib/hsv2rgb.h"
#include "lib/lights_controller.h"

#include "chprintf.h"
#include <optional>

namespace commands {
namespace {

std::optional<bool> parse_bool_flag(const char* command_name, BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc != 1) {
    chprintf(chp, "Usage: %s on/off\r\n", command_name);
    return std::nullopt;
  }
  if (argv[0] == std::string_view("on")) {
    return true;
  } else if (argv[0] == std::string_view("off")) {
    return false;
  }

  chprintf(chp, "Usage: %s on/off\r\n", command_name);
  return std::nullopt;
}

} // namespace

void pwm(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc != 2) {
    chprintf(chp, "Usage: pwm channel duty_cycle\r\n");
    return;
  }

  int channel = atoi(argv[0]);
  int duty_cycle = atoi(argv[1]);

  if (channel < 0 || channel > 7) {
    chprintf(chp, "Unsupported channel.\r\n");
    return;
  }

  if (duty_cycle < 0 || duty_cycle > MAX_BRIGHTNESS) {
    chprintf(chp, "Duty cycle out of range.\r\n");
    return;
  }

  setPwm(channel, duty_cycle);
}

void rgb(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc != 4) {
    chprintf(chp, "Usage: rgb channel r g b\r\n");
    return;
  }

  int channel = atoi(argv[0]);
  int r = atoi(argv[1]);
  int g = atoi(argv[2]);
  int b = atoi(argv[3]);

  if (channel < 0 || channel >= 110) {
    chprintf(chp, "Unsupported channel.\r\n");
    return;
  }

  ws2812_write_led(channel, r, g, b);
}

void hsv(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc != 4) {
    chprintf(chp, "Usage: hsv channel h[0..360] s[0..256] v[0..256]\r\n");
    return;
  }

  int channel = atoi(argv[0]);
  float h = atoi(argv[1]);
  float s = atoi(argv[2]) / 256.;
  float v = atoi(argv[3]) / 256.;

  if (channel < 0 || channel >= 110) {
    chprintf(chp, "Unsupported channel.\r\n");
    return;
  }

  auto rgb = HSVtoRGB(h, s, v);
  ws2812_write_led(channel, rgb.r, rgb.g, rgb.b);
}

void pixie_lights(BaseSequentialStream *chp, int argc, char *argv[]) {
  std::optional<bool> flag = parse_bool_flag("pixie_lights", chp, argc, argv);
  if (flag) {
    lights_controller()->set_pixie_on(*flag);
  }
}

void rgb_backlight(BaseSequentialStream *chp, int argc, char *argv[]) {
  std::optional<bool> flag = parse_bool_flag("tree_backlight", chp, argc, argv);
  if (flag) {
    lights_controller()->set_rgb_on(*flag);
  }
}

void glimmer(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc != 1) {
    chprintf(chp, "Usage: glimmer speed[0..3]\r\n");
    return;
  }

  lights_controller()->set_glimmer_speed(atoi(argv[0]));
}

void brightness(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc != 1) {
    chprintf(chp, "Usage: brightness b\r\n");
    return;
  }

  int new_brightness = atoi(argv[0]);
  if (new_brightness > MAX_BRIGHTNESS) {
    chprintf(chp, "Brighntness too high.\r\n");
    return;
  }
  if (new_brightness < MIN_BRIGHTNESS) {
    chprintf(chp, "Brighntness too low.\r\n");
    return;
  }

  lights_controller()->set_brightness(new_brightness);
}

void play(BaseSequentialStream *chp, int /*argc*/, char* /*argv*/[]) {
  execute_lamp_asm(chp);
}

} // namespace commands
