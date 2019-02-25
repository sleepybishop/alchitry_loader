#ifndef JTAG_H_
#define JTAG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ftdi.h>
#include <stdbool.h>
#include <unistd.h>

#include "jtag_fsm.h"

struct jtag_ctx {
  struct ftdi_context *ftdi;
  unsigned int ui_dev_index;
  bool active;
};

bool jtag_sync_mpsse(struct jtag_ctx *jtag);
bool jtag_config(struct jtag_ctx *jtag);

struct jtag_ctx *jtag_new();
void jtag_shutdown(struct jtag_ctx *jtag);
bool jtag_initialize(struct jtag_ctx *jtag);
bool jtag_set_freq(struct jtag_ctx *jtag, double freq);
bool jtag_navigate_to_state(struct jtag_ctx *jtag, enum jtag_fsm_state init,
                            enum jtag_fsm_state dest);
bool jtag_shift_data(struct jtag_ctx *jtag, unsigned int, char *, char *,
                     char *);
char *jtag_shift_data2(struct jtag_ctx *jtag, unsigned int, char *);
bool jtag_send_clocks(struct jtag_ctx *jtag, unsigned long cycles);

#ifdef __cplusplus
}
#endif
#endif /* JTAG_H_ */
