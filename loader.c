#include "loader.h"
#include "jtag.h"
#include <stdio.h>
#include <unistd.h>

static bool loader_set_IR(struct loader_ctx *loader, enum instruction);
static bool loader_shift_DR(struct loader_ctx *loader, int bits, char *write,
                            char *read, char *mask, bool from_file);
static bool loader_shift_IR(struct loader_ctx *loader, int, char *, char *,
                            char *);
static bool loader_load_bin(struct loader_ctx *loader, char *file);
static bool loader_set_state(struct loader_ctx *loader,
                             enum jtag_fsm_state state);

struct loader_ctx *loader_new(struct jtag_ctx *dev) {
  struct loader_ctx *loader = calloc(1, sizeof(struct loader_ctx));
  loader->device = dev;
  loader->current_state = TEST_LOGIC_RESET;
  return loader;
}

bool loader_set_state(struct loader_ctx *loader, enum jtag_fsm_state state) {
  if (!jtag_navigate_to_state(loader->device, loader->current_state, state)) {
    return false;
  }
  loader->current_state = state;
  return true;
}

bool loader_reset_state(struct loader_ctx *loader) {
  loader->current_state = TEST_LOGIC_RESET;
  return jtag_navigate_to_state(loader->device, CAPTURE_DR, TEST_LOGIC_RESET);
}

bool loader_set_IR(struct loader_ctx *loader, enum instruction inst) {
  char inst_str[8];
  sprintf(inst_str, "%02x", inst);

  if (!jtag_navigate_to_state(loader->device, loader->current_state,
                              SHIFT_IR)) {
    fprintf(stderr, "Failed to change to SHIFT_IR state!\n");
    return false;
  }
  if (!jtag_shift_data(loader->device, 6, inst_str, "", "", false)) {
    fprintf(stderr, "Failed to shift instruction data!\n");
    return false;
  }
  if (!jtag_navigate_to_state(loader->device, EXIT1_IR, RUN_TEST_IDLE)) {
    fprintf(stderr, "Failed to change to RUN_TEST_IDLE state!\n");
    return false;
  }
  loader->current_state = RUN_TEST_IDLE;
  return true;
}

bool loader_shift_DR(struct loader_ctx *loader, int bits, char *write,
                     char *read, char *mask, bool from_file) {
  if (!jtag_navigate_to_state(loader->device, loader->current_state,
                              SHIFT_DR)) {
    fprintf(stderr, "Failed to change to SHIFT_DR state!\n");
    return false;
  }
  if (!jtag_shift_data(loader->device, bits, write, read, mask, from_file)) {
    fprintf(stderr, "Failed to shift data!\n");
    return false;
  }
  if (!jtag_navigate_to_state(loader->device, EXIT1_DR, RUN_TEST_IDLE)) {
    fprintf(stderr, "Failed to change to RUN_TEST_IDLE state!\n");
    return false;
  }
  loader->current_state = RUN_TEST_IDLE;
  return true;
}

bool loader_shift_IR(struct loader_ctx *loader, int bits, char *write,
                     char *read, char *mask) {
  if (!jtag_navigate_to_state(loader->device, loader->current_state,
                              SHIFT_IR)) {
    fprintf(stderr, "Failed to change to SHIFT_IR state!\n");
    return false;
  }
  if (!jtag_shift_data(loader->device, bits, write, read, mask, false)) {
    fprintf(stderr, "Failed to shift data!\n");
    return false;
  }
  if (!jtag_navigate_to_state(loader->device, EXIT1_IR, RUN_TEST_IDLE)) {
    fprintf(stderr, "Failed to change to RUN_TEST_IDLE state!\n");
    return false;
  }
  loader->current_state = RUN_TEST_IDLE;
  return true;
}

bool loader_load_bin(struct loader_ctx *loader, char *file) {
  if (!jtag_set_freq(loader->device, 10000000)) {
    fprintf(stderr, "Failed to set JTAG frequency!\n");
    return false;
  }
  if (!loader_reset_state(loader))
    return false;
  if (!loader_set_state(loader, RUN_TEST_IDLE))
    return false;

  if (!loader_set_IR(loader, JPROGRAM))
    return false;
  if (!loader_set_IR(loader, ISC_NOOP))
    return false;

  usleep(100000);

  // config/jprog/poll
  if (!jtag_send_clocks(loader->device, 10000))
    return false;
  if (!loader_shift_IR(loader, 6, "14", "11", "31"))
    return false;

  // config/slr
  if (!loader_set_IR(loader, CFG_IN))
    return false;
  if (!loader_shift_DR(loader, 1, file, "", "", true)) {
    return false;
  }

  // config/start
  if (!loader_set_state(loader, RUN_TEST_IDLE))
    return false;
  if (!jtag_send_clocks(loader->device, 100000))
    return false;
  if (!loader_set_IR(loader, JSTART))
    return false;
  if (!loader_set_state(loader, RUN_TEST_IDLE))
    return false;
  if (!jtag_send_clocks(loader->device, 100))
    return false;
  if (!loader_shift_IR(loader, 6, "09", "31", "11"))
    return false;

  // config/status
  if (!loader_set_state(loader, TEST_LOGIC_RESET))
    return false;
  if (!jtag_send_clocks(loader->device, 5))
    return false;
  if (!loader_set_IR(loader, CFG_IN))
    return false;
  if (!loader_shift_DR(loader, 160, "0000000400000004800700140000000466aa9955",
                       "", "", false))
    return false;
  if (!loader_set_IR(loader, CFG_OUT))
    return false;
  if (!loader_shift_DR(loader, 32, "00000000", "3f5e0d40", "08000000", false))
    return false;
  if (!loader_set_state(loader, TEST_LOGIC_RESET))
    return false;
  if (!jtag_send_clocks(loader->device, 5))
    return false;

  return true;
}

bool loader_erase_flash(struct loader_ctx *loader, char *loader_file) {
  fprintf(stdout, "Initializing FPGA...\n");
  if (!loader_load_bin(loader, loader_file)) {
    fprintf(stdout, "Failed to initialize FPGA!\n");
    return false;
  }

  if (!jtag_set_freq(loader->device, 1500000)) {
    fprintf(stderr, "Failed to set JTAG frequency!\n");
    return false;
  }

  fprintf(stdout, "Erasing...\n");

  // Erase the flash
  if (!loader_set_IR(loader, USER1))
    return false;

  if (!loader_shift_DR(loader, 1, "0", "", "", false))
    return false;

  usleep(10000000);

  if (!loader_set_IR(loader, JPROGRAM))
    return false;

  // reset just for good measure
  if (!loader_reset_state(loader)) {
    return false;
  }

  return true;
}

bool loader_write_bin(struct loader_ctx *loader, char *bin_file, bool flash,
                      char *loader_file) {
  if (flash) {
    fprintf(stdout, "Initializing FPGA...\n");
    if (!loader_load_bin(loader, loader_file)) {
      fprintf(stderr, "Failed to initialize FPGA!\n");
      return false;
    }

    if (!jtag_set_freq(loader->device, 1500000)) {
      fprintf(stderr, "Failed to set JTAG frequency!\n");
      return false;
    }

    fprintf(stdout, "Erasing...\n");

    // Erase the flash
    if (!loader_set_IR(loader, USER1))
      return false;

    if (!loader_shift_DR(loader, 0, "0", "", "", false))
      return false;

    usleep(100000);

    fprintf(stdout, "Writing...\n");

    // Write the flash
    if (!loader_set_IR(loader, USER2))
      return false;

    if (!loader_shift_DR(loader, 0, bin_file, "", "", true)) {
      return false;
    }

    // If you enter the reset state after a write
    // the loader firmware resets the flash into
    // regular SPI mode and gets stuck in a dead FSM
    // state. You need to do this before issuing a
    // JPROGRAM command or the FPGA can't read the
    // flash.
    if (!loader_reset_state(loader))
      return false;

    usleep(100000); // 100ms delay is required before issuing JPROGRAM

    fprintf(stdout, "Resetting FPGA...\n");
    // JPROGRAM resets the FPGA configuration and will
    // cause it to read the flash memory
    if (!loader_set_IR(loader, JPROGRAM))
      return false;
  } else {
    fprintf(stdout, "Programming FPGA...\n");
    if (!loader_load_bin(loader, bin_file)) {
      fprintf(stderr, "Failed to initialize FPGA!\n");
      return false;
    }
  }

  // reset just for good measure
  if (!loader_reset_state(loader))
    return false;

  fprintf(stdout, "Done.\n");
  return true;
}

bool loader_check_IDCODE(struct loader_ctx *loader) {
  if (!loader_set_IR(loader, IDCODE))
    return false;

  if (!loader_shift_DR(loader, 32, "00000000", "0362D093",
                       "0FFFFFFF", false)) // FPGA IDCODE
    return false;

  return true;
}
