# Photon charmer - art lamp controller.

My hobby is making lamps. Couple of latest projects required more advanced control than a mechanical switch. This repository holds firmware for these lamps. It runs on blue pill boards (STM32F103CB based) and under Chibi OS.

## Hydrangea coffee table.

[![Coffee table](/images/table_small.jpg)](https://raw.githubusercontent.com/kibergus/photon_charmer/main/images/table.jpg)
[See video of it in action.](https://youtu.be/-L07Zq1sDZU)

Design by Anastasia Petrushkina, electronics and firmware by Alexey Guseynov.

Materials: hydrangea petals, resin, brass, acrylic paints, ws2812b.

## Pixie tree.
[![Pixie tree](/images/tree_lamp_small.jpg)](https://youtu.be/gSbW40JnVsY)
[See video of it in action.](https://youtu.be/gSbW40JnVsY)

Design by Alexey Guseynov.

Tree: brass wire, pixie lights.

Stand: wood, cardboard, modelling clay, pixie lights, ws2812b

# Main features:

* 8 channel PWM controller with logariphmic dimming curve.
  * Perception-linear brightness due to logariphmic curve.
  * 1kHZ rate: no visible flickering.
  * 50000 brightness levels allow very low minimal brightness. This is important for ambient light especially if logariphmic dimming curve is used.
  * Timer based, doesn't waste CPU cicles.
* ws2812 RGB LED controller.
  * DMA based, doesn't waste CPU.
* Button controller.
  * KeyUp, KeyDow, Click, LongPress, KeyHeld events.
  * Interrupt based. 
* 4 channel touch controller.
  * No external circuosity required. Just connect sensing pads to GPIO pin.
* USB shell.
* Basic built-in effects.
* USB streaming effects support.

# Pinout

PWM channels: TIM3, TIM4 -> A6, A7, B0, B1, B6, B7, B8, B9
ws2812: TIM1 -> A8
ZigBee (via TRÅDFRI module): A4, A5

**Tree specific:**
Buttons: A0 A1

**Table specific:**
Touch sensors: A0 A1 A2 A3

# PS

This is a hobby project with relatively simple functionality. Making firmware up to all standards was not a target. No unit tests, code may be ugly. But it works and brightens our evenings.
