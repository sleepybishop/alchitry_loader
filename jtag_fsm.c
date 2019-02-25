#include "jtag_fsm.h"

#define QSZ 64

enum jtag_fsm_state get_transition(enum jtag_fsm_state state, bool tms) {
  switch (state) {
  case TEST_LOGIC_RESET:
    return tms ? TEST_LOGIC_RESET : RUN_TEST_IDLE;
  case RUN_TEST_IDLE:
    return tms ? SELECT_DR_SCAN : RUN_TEST_IDLE;
  case SELECT_DR_SCAN:
    return tms ? SELECT_IR_SCAN : CAPTURE_DR;
  case CAPTURE_DR:
    return tms ? EXIT1_DR : SHIFT_DR;
  case SHIFT_DR:
    return tms ? EXIT1_DR : SHIFT_DR;
  case EXIT1_DR:
    return tms ? UPDATE_DR : PAUSE_DR;
  case PAUSE_DR:
    return tms ? EXIT2_DR : PAUSE_DR;
  case EXIT2_DR:
    return tms ? UPDATE_DR : SHIFT_DR;
  case UPDATE_DR:
    return tms ? SELECT_DR_SCAN : RUN_TEST_IDLE;
  case SELECT_IR_SCAN:
    return tms ? TEST_LOGIC_RESET : CAPTURE_IR;
  case CAPTURE_IR:
    return tms ? EXIT1_IR : SHIFT_IR;
  case SHIFT_IR:
    return tms ? EXIT1_IR : SHIFT_IR;
  case EXIT1_IR:
    return tms ? UPDATE_IR : PAUSE_IR;
  case PAUSE_IR:
    return tms ? EXIT2_IR : PAUSE_IR;
  case EXIT2_IR:
    return tms ? UPDATE_IR : SHIFT_IR;
  case UPDATE_IR:
    return tms ? SELECT_DR_SCAN : RUN_TEST_IDLE;
  }
  return TEST_LOGIC_RESET;
}

struct jtag_fsm_transitions get_transitions(enum jtag_fsm_state init,
                                            enum jtag_fsm_state final) {
  struct jtag_fsm_transitions queue[QSZ] = {0};
  int qs = 0, qh = 0, qt = 0;
  struct jtag_fsm_transitions t = {init, 0, 0};

  queue[0] = t;
  qs++;

  while (qs > 0) {
    t = queue[qh];
    qh = (qh + 1) % QSZ;
    qs--;

    if (t.current_state == final) {
      break;
    }

    enum jtag_fsm_state s0 = get_transition(t.current_state, false);
    enum jtag_fsm_state s1 = get_transition(t.current_state, true);

    if ((qs + 1) >= QSZ)
      abort();

    t.moves++;
    t.tms &= ~(1 << (t.moves - 1)); // clear bit
    t.current_state = s0;
    qt = (qt + 1) % QSZ;
    queue[qt] = t;
    qs++;

    t.tms |= (1 << (t.moves - 1));
    t.current_state = s1;
    qt = (qt + 1) % QSZ;
    queue[qt] = t;
    qs++;
  }
  return t;
}

const char *get_state_name(enum jtag_fsm_state state) {
  switch (state) {
  case TEST_LOGIC_RESET:
    return "RESET";
  case RUN_TEST_IDLE:
    return "IDLE";
  case SELECT_DR_SCAN:
    return "DRSELECT";
  case CAPTURE_DR:
    return "DRCAPTURE";
  case SHIFT_DR:
    return "DRSHIFT";
  case EXIT1_DR:
    return "DREXIT1";
  case PAUSE_DR:
    return "DRPAUSE";
  case EXIT2_DR:
    return "DREXIT2";
  case UPDATE_DR:
    return "DRUPDATE";
  case SELECT_IR_SCAN:
    return "IRSELECT";
  case CAPTURE_IR:
    return "IRCAPTURE";
  case SHIFT_IR:
    return "IRSHIFT";
  case EXIT1_IR:
    return "IREXIT1";
  case PAUSE_IR:
    return "IRPAUSE";
  case EXIT2_IR:
    return "IREXIT2";
  case UPDATE_IR:
    return "IRUPDATE";
  default:
    fprintf(stderr, "Unknown fsm state\n");
    return NULL;
  }
}

enum jtag_fsm_state get_state_from_name(char *name) {
  if (0 == strcmp(name, "RESET"))
    return TEST_LOGIC_RESET;
  if (0 == strcmp(name, "IDLE"))
    return RUN_TEST_IDLE;
  if (0 == strcmp(name, "DRSELECT"))
    return SELECT_DR_SCAN;
  if (0 == strcmp(name, "DRCAPTURE"))
    return CAPTURE_DR;
  if (0 == strcmp(name, "DRSHIFT"))
    return SHIFT_DR;
  if (0 == strcmp(name, "DREXIT1"))
    return EXIT1_DR;
  if (0 == strcmp(name, "DRPAUSE"))
    return PAUSE_DR;
  if (0 == strcmp(name, "DREXIT2"))
    return EXIT2_DR;
  if (0 == strcmp(name, "DRUPDATE"))
    return UPDATE_DR;
  if (0 == strcmp(name, "IRSELECT"))
    return SELECT_IR_SCAN;
  if (0 == strcmp(name, "IRCAPTURE"))
    return CAPTURE_IR;
  if (0 == strcmp(name, "IRSHIFT"))
    return SHIFT_IR;
  if (0 == strcmp(name, "IREXIT1"))
    return EXIT1_IR;
  if (0 == strcmp(name, "IRPAUSE"))
    return PAUSE_IR;
  if (0 == strcmp(name, "IREXIT2"))
    return EXIT2_IR;
  if (0 == strcmp(name, "IRUPDATE"))
    return UPDATE_IR;

  fprintf(stderr, "ERROR! Invalid state name: %s \n", name);
  return TEST_LOGIC_RESET;
}
