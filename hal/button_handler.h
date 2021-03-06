#pragma once

#include "ch.h"
#include "hal.h"

constexpr int CLICK_TIME_MS = 300;
constexpr int LONG_PRESS_TIME_MS = 1000;

// Event handler interface.
class ButtonHandler {
public:
  virtual void onKeyDown() {}
  // Called every key_hold_interval while key is held pressed.
  virtual void onKeyHeldDown() {}
  virtual void onKeyUp() {}
  virtual void onClick() {}
  virtual void onLongPress() {}
};

// Recognizes high level events like clicks, button hold or long press. Allows to bind multiple actions to a single button.
class ButtonController {
public:
  ButtonController(ioportid_t port, iopadid_t pad, sysinterval_t key_hold_interval, ButtonHandler* handler);

  // Invoke this function from a worker thread.
  void __attribute__((noreturn)) operator()();

private:
  msg_t waitState(bool state, sysinterval_t timeout);

  ioportid_t port_;
  iopadid_t pad_;
  sysinterval_t key_hold_interval_;
  ButtonHandler* handler_;
};
