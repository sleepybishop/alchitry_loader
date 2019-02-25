#ifndef JTAG_FSM_H_
#define JTAG_FSM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum jtag_fsm_state {
  TEST_LOGIC_RESET,
  RUN_TEST_IDLE,
  SELECT_DR_SCAN,
  CAPTURE_DR,
  SHIFT_DR,
  EXIT1_DR,
  PAUSE_DR,
  EXIT2_DR,
  UPDATE_DR,
  SELECT_IR_SCAN,
  CAPTURE_IR,
  SHIFT_IR,
  EXIT1_IR,
  PAUSE_IR,
  EXIT2_IR,
  UPDATE_IR
};

struct jtag_fsm_transitions {
  enum jtag_fsm_state current_state;
  uint8_t tms;
  uint8_t moves;
};

struct jtag_fsm_transitions get_transitions(enum jtag_fsm_state init,
                                            enum jtag_fsm_state final);
const char *get_state_name(enum jtag_fsm_state state);
enum jtag_fsm_state get_state_from_name(char *name);
enum jtag_fsm_state get_transition(enum jtag_fsm_state state, bool tms);

#ifdef __cplusplus
}
#endif
#endif /* JTAG_FSM_H_ */
