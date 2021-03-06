#include "button_handler.h"
#include <utility>

ButtonController::ButtonController(ioportid_t port, iopadid_t pad, sysinterval_t key_hold_interval, ButtonHandler* handler) 
    : port_(port)
    , pad_(pad)
    , key_hold_interval_(key_hold_interval)
    , handler_(std::move(handler)) {
  palSetPadMode(port_, pad_, PAL_MODE_INPUT_PULLUP);
  palEnablePadEvent(port_, pad_, PAL_EVENT_MODE_BOTH_EDGES);
}

void __attribute__((noreturn)) ButtonController::operator()() {
  while (true) {
    waitState(PAL_LOW, TIME_INFINITE);
    systime_t buttonDownTime = chVTGetSystemTimeX();
    handler_->onKeyDown();

    bool long_press_reported = false;
    while (true) {
      if (waitState(PAL_HIGH, key_hold_interval_) == MSG_OK) {
        break;
      }
      sysinterval_t bottonHoldTime = chVTTimeElapsedSinceX(buttonDownTime);

      if (!long_press_reported && bottonHoldTime > TIME_MS2I(LONG_PRESS_TIME_MS)) {
        handler_->onLongPress();
        long_press_reported = true;
      }
      if (bottonHoldTime > TIME_MS2I(CLICK_TIME_MS)) {
          handler_->onKeyHeldDown();
      }
    }

    sysinterval_t bottonHoldTime = chVTTimeElapsedSinceX(buttonDownTime);
    handler_->onKeyUp();

    if (TIME_I2MS(bottonHoldTime) < CLICK_TIME_MS) {
      handler_->onClick();
    }
  }
}

msg_t ButtonController::waitState(bool state, sysinterval_t timeout) {
  for (;;) {
    chSysLock();
    if (palReadPad(port_, pad_) == state) {
      chSysUnlock();
      return MSG_OK;
    }
    if (palWaitPadTimeoutS(port_, pad_, timeout) == MSG_TIMEOUT) {
      chSysUnlock();
      return MSG_TIMEOUT;
    }
    chSysUnlock();

    chThdSleepMilliseconds(10);    
  }
}
