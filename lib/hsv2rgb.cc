#include "hsv2rgb.h"

#include <stdint.h>
#include <cmath>

RGB HSVtoRGB(float h, float s, float v) {
  float chroma = v * s;
  float h_prime = std::fmod(h / 60.0, 6);
  float x = chroma * (1 - std::fabs(std::fmod(h_prime, 2) - 1));
  
  float r = 0;
  float g = 0;
  float b = 0;

  if(h_prime < 1) {
    r = chroma;
    g = x;
    b = 0;
  } else if(h_prime < 2) {
    r = x;
    g = chroma;
    b = 0;
  } else if(h_prime < 3) {
    r = 0;
    g = chroma;
    b = x;
  } else if(h_prime < 4) {
    r = 0;
    g = x;
    b = chroma;
  } else if(h_prime < 5) {
    r = x;
    g = 0;
    b = chroma;
  } else if(h_prime < 6) {
    r = chroma;
    g = 0;
    b = x;
  }

  r += v - chroma;
  g += v - chroma;
  b += v - chroma;

  return {uint8_t(r * 255), uint8_t(g * 255), uint8_t(b * 255)};
}
