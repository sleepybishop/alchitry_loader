#ifndef LOADER_H_
#define LOADER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "jtag.h"
#include "jtag_fsm.h"
#include <string.h>

enum instruction {
  EXTEST = 0x26,
  EXTEST_PULSE = 0x3C,
  EXTEST_TRAIN = 0x3D,
  SAMPLE = 0x01,
  USER1 = 0x02,
  USER2 = 0x03,
  USER3 = 0x22,
  USER4 = 0x23,
  CFG_OUT = 0x04,
  CFG_IN = 0x05,
  USERCODE = 0x08,
  IDCODE = 0x09,
  HIGHZ_IO = 0x0A,
  JPROGRAM = 0x0B,
  JSTART = 0x0C,
  JSHUTDOWN = 0x0D,
  XADC_DRP = 0x37,
  ISC_ENABLE = 0x10,
  ISC_PROGRAM = 0x11,
  XSC_PROGRAM_KEY = 0x12,
  XSC_DNA = 0x17,
  FUSE_DNA = 0x32,
  ISC_NOOP = 0x14,
  ISC_DISABLE = 0x16,
  BYPASS = 0x2F,
};

struct loader_ctx {
  struct jtag_ctx *device;
  enum jtag_fsm_state current_state;
};

bool loader_set_IR(struct loader_ctx *loader, enum instruction);
bool loader_shift_DR(struct loader_ctx *loader, int bits, char *write,
                     char *read, char *mask);
bool loader_shift_IR(struct loader_ctx *loader, int, char *, char *, char *);

bool loader_load_bin(struct loader_ctx *loader, char *file);
bool loader_set_state(struct loader_ctx *loader, enum jtag_fsm_state state);

struct loader_ctx *loader_new(struct jtag_ctx *jtag);
bool loader_reset_state(struct loader_ctx *loader);
bool loader_check_IDCODE(struct loader_ctx *loader);
bool loader_erase_flash(struct loader_ctx *loader, char *loader_file);
bool loader_write_bin(struct loader_ctx *loader, char *bin_file, bool flash,
                      char *loader_file);

#ifdef __cplusplus
}
#endif
#endif /* LOADER_H_ */
