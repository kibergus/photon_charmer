#pragma once

constexpr int MAX_BRIGHTNESS = 40000;
constexpr int MIN_BRIGHTNESS = 1;

// 8 channel PWM Controller. Channels are mapped to following pins:
// A6, A7, B0, B1, B6, B7, B8, B9
// A logariphmic dimming curve is applied to provide linearity at perception level.
void initPwm();
void setPwm(int channel, int brightness);
