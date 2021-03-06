#include "asm.h"

#include "hal/pwm.h"
#include "hal/ws2812/ws2812.h"

namespace {

constexpr int COMMAND_SIZE = 9;
constexpr int BUFFER_SIZE = 250;

uint8_t buffer[BUFFER_SIZE][COMMAND_SIZE];
// Instruction pointer.
volatile int ip = 0;
int read_pointer = 0;


THD_WORKING_AREA(asmThreadArea, 256);
__attribute__((noreturn)) THD_FUNCTION(asmThread, arg) {
  (void)arg;
  chRegSetThreadName("asm");

  // Timer has a 16 bit resolution and we need more.
  int elapsed = 0;
  systime_t last_timer_value = chVTGetSystemTimeX();

  for (;;) {
    uint8_t* command = buffer[ip];
    uint8_t opcode = command[0];
    if (opcode == 0) {
      chThdExit(0);
    }
    if (opcode == 1) {
      int delay = (int(command[1]) << 24) |
                  (int(command[2]) << 16) |
                  (int(command[3]) << 8) |
                  int(command[4]);
      systime_t next_timer_value = chVTGetSystemTimeX();
      elapsed += chTimeDiffX(last_timer_value, next_timer_value);
      last_timer_value = next_timer_value;

      // 2 is a magic constant. For some reason it is just 2 times faster than should.
      if (delay > int(TIME_I2MS(elapsed)) * 2) {
        chThdSleepMilliseconds(delay - elapsed);
      }
    }
    if (opcode == 2 || opcode == 3) {
      // PWM command.
      for (int offset = 0; offset < 4; ++offset) {
        int channel = offset + (opcode == 2 ? 0 : 4);
        int duty_cycle = int(command[offset * 2 + 1]) << 8 | command[offset * 2 + 2];
        setPwm(channel, duty_cycle);
      }
    }
    if (opcode >= 4 || opcode <= 8) {
      // RGB command.
      for (int offset = 0; offset < 1; ++offset) {
        int channel = offset + (opcode - 4) * 2;
        ws2812_write_led(channel, command[offset * 3 + 1], command[offset * 3 + 2], command[offset * 3 + 3]);
      }
    }

    ip = (ip + 1) % BUFFER_SIZE;
  }
}

uint8_t decode_hex(uint8_t c) {
  if (c <= '9') {
    return c - '0';
  } else {
    return c - 'a' + 10;
  }
}

void read_command(BaseSequentialStream *chp) {
  uint8_t hex_buffer[COMMAND_SIZE * 2];
  for (int i = 0; i < COMMAND_SIZE * 2; ++i) {
    hex_buffer[i] = streamGet(chp);
  }
  for (int i = 0; i < COMMAND_SIZE; ++i) {
    buffer[read_pointer][i] = decode_hex(hex_buffer[i * 2]) << 4 | decode_hex(hex_buffer[i * 2 + 1]);
  }
}

} // namespace

void execute_lamp_asm(BaseSequentialStream* chp) {
  ip = 0;
  read_pointer = 0;

  for (;;) {
    read_command(chp);
    if (buffer[read_pointer][0] == 0 || buffer[read_pointer][0] == 1) {
      break;
    }
    read_pointer = (read_pointer + 1) % BUFFER_SIZE;
  }

  chThdCreateStatic(asmThreadArea, sizeof(asmThreadArea), NORMALPRIO, asmThread, NULL);

  for (;;) {
    while (read_pointer == ip) {
      chThdSleepMilliseconds(1);
    }

    read_command(chp);
    if (buffer[read_pointer][0] == 0) {
      return;
    }
    read_pointer = (read_pointer + 1) % BUFFER_SIZE;
  }
}
