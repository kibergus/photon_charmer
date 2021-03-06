// A collection of commands for ChibiOS shell.
#pragma once

#include "hal.h"

namespace commands {

// Low level controls.
void pwm(BaseSequentialStream *chp, int argc, char *argv[]);
void rgb(BaseSequentialStream *chp, int argc, char *argv[]);
void hsv(BaseSequentialStream *chp, int argc, char *argv[]);

// Light asm interpreter.
void play(BaseSequentialStream *chp, int /*argc*/, char* /*argv*/[]);

// LightsController interface.
void pixie_lights(BaseSequentialStream *chp, int argc, char *argv[]);
void rgb_backlight(BaseSequentialStream *chp, int argc, char *argv[]);
void glimmer(BaseSequentialStream *chp, int argc, char *argv[]);
void brightness(BaseSequentialStream *chp, int argc, char *argv[]);

} // namespace commands
