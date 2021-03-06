#pragma once

struct RGB {
  unsigned char r;
  unsigned char g;
  unsigned char b;
};

// Converts HSV (0..360, 0..1, 0..1) to RGB needed by ws2812 LEDs.
RGB HSVtoRGB(float h, float s, float v);
