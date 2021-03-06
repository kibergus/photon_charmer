#include "ch.h"
#include "hal.h"

#include "usb/usbcfg.h"

static THD_WORKING_AREA(blinkerThreadArea, 128);
static __attribute__((noreturn)) THD_FUNCTION(blinkerThread, arg) {
  (void)arg;
  chRegSetThreadName("blinker");

  while (true) {
    systime_t time = serusbcfg.usbp->state == USB_ACTIVE ? 250 : 500;
    palClearPad(GPIOC, 13);
    chThdSleepMilliseconds(time);
    palSetPad(GPIOC, 13);
    chThdSleepMilliseconds(time);
  }
}

void startBlinker() {
  palSetPadMode(GPIOC, 13, PAL_MODE_OUTPUT_PUSHPULL);
  chThdCreateStatic(blinkerThreadArea, sizeof(blinkerThreadArea), NORMALPRIO, blinkerThread, NULL);
}
