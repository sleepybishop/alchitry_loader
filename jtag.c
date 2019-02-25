#include "jtag.h"
#include <string.h>
#include <unistd.h>

#define LATENCY_MS 16
#define CHUNK_SIZE 65535
#define USB_TIMEOUT 5000

static unsigned char byte_from_hex_string(char *hex, unsigned int offset,
                                          unsigned int num) {
  char tmp[3] = {0, 0, 0};
  tmp[0] = hex[offset];
  if (num == 2) {
    tmp[1] = hex[offset + 1];
  }
  return strtol(tmp, NULL, 16);
}

static bool compare_hex_string(char *a, char *b, char *mask) {
  unsigned int a_len = strlen(a);
  unsigned int b_len = strlen(b);

  if (a_len != b_len) {
    fprintf(stdout, "length mismatch!\n");
    return false;
  }

  if (mask == NULL || strlen(mask) == 0)
    return (strcmp(a, b) == 0);

  if (a_len != strlen(mask)) {
    fprintf(stdout, "length and mask mismatch!\n");
    return false;
  }

  for (unsigned int i = 0; i < a_len / 2; i++) {
    unsigned char m = byte_from_hex_string(mask, a_len - 2 - i * 2, 2);
    unsigned char pa = byte_from_hex_string(a, a_len - 2 - i * 2, 2);
    unsigned char pb = byte_from_hex_string(b, a_len - 2 - i * 2, 2);
    if ((pa & m) != (pb & m)) {
      fprintf(stdout, "Mismatch at %d\n", i);
      return false;
    }
  }
  if ((a_len & 1) != 0) {
    unsigned char m = byte_from_hex_string(mask, 0, 1);
    unsigned char pa = byte_from_hex_string(a, 0, 1);
    unsigned char pb = byte_from_hex_string(b, 0, 1);
    if ((pa & m) != (pb & m)) {
      fprintf(stdout, "Mismatch at last bit\n");
      return false;
    }
  }

  return true;
}

struct jtag_ctx *jtag_new(struct ftdi_context *ftdi) {
  struct jtag_ctx *ctx = calloc(1, sizeof(struct jtag_ctx));

  ctx->ftdi = ftdi;
  ctx->active = false;
  ctx->ui_dev_index = 0xF;

  return ctx;
}

bool jtag_initialize(struct jtag_ctx *jtag) {
  int status = 0;
  status |= ftdi_usb_reset(jtag->ftdi);
  status |= ftdi_set_latency_timer(jtag->ftdi, LATENCY_MS);
  status |= ftdi_write_data_set_chunksize(jtag->ftdi, CHUNK_SIZE);
  status |= ftdi_read_data_set_chunksize(jtag->ftdi, CHUNK_SIZE);
  status |= ftdi_set_bitmode(jtag->ftdi, 0, BITMODE_RESET);
  status |= ftdi_set_bitmode(jtag->ftdi, 0, BITMODE_MPSSE);

  jtag->ftdi->usb_read_timeout = USB_TIMEOUT;
  jtag->ftdi->usb_write_timeout = USB_TIMEOUT;

  if (status != 0) {
    fprintf(stderr, "Failed to set initial configuration!\n");
    return false;
  }

  usleep(100000);
  ftdi_usb_purge_buffers(jtag->ftdi);

  if (!jtag_sync_mpsse(jtag)) {
    fprintf(stderr, "Failed to sync with MPSSE!\n");
    return false;
  }

  if (!jtag_config(jtag)) {
    fprintf(stderr, "Failed to set JTAG configuration!\n");
    return false;
  }

  jtag->active = true;

  return true;
}

void jtag_shutdown(struct jtag_ctx *jtag) {
  if (jtag) {
    if (jtag->active) {
      ftdi_set_bitmode(jtag->ftdi, 0, BITMODE_RESET);
      ftdi_usb_close(jtag->ftdi);
      ftdi_deinit(jtag->ftdi);
    }
    free(jtag);
    jtag = NULL;
  }
}

bool jtag_sync_mpsse(struct jtag_ctx *jtag) {
  unsigned char cmd[2] = {0xaa, 0x0};
  int cmdlen = 1;

  if (cmdlen != ftdi_write_data(jtag->ftdi, cmd, cmdlen)) {
    fprintf(stderr, "Failed to send bad command\n");
  }

  int n = 0, r = 0;
  while (n < cmdlen) {
    r = ftdi_read_data(jtag->ftdi, cmd, cmdlen);
    if (r < 0)
      break;
    n += r;
  }
  ftdi_usb_purge_rx_buffer(jtag->ftdi);

  return n == cmdlen;
}

bool jtag_config(struct jtag_ctx *jtag) {
  unsigned char cmd[3];
  int cmdlen = sizeof(cmd);
  int divisor = 0x05DB;

  cmd[0] = DIS_DIV_5;
  cmd[1] = DIS_ADAPTIVE;
  cmd[2] = DIS_3_PHASE;
  if (cmdlen != ftdi_write_data(jtag->ftdi, cmd, cmdlen)) {
    fprintf(stderr, "Failed to send speed command\n");
    return false;
  }

  cmd[0] = SET_BITS_LOW;
  cmd[1] = 0x08;
  cmd[2] = 0x0b;
  if (cmdlen != ftdi_write_data(jtag->ftdi, cmd, cmdlen)) {
    fprintf(stderr, "Failed to send low gpio command\n");
    return false;
  }

  cmd[0] = SET_BITS_HIGH;
  cmd[1] = 0x0;
  cmd[2] = 0x0;
  if (cmdlen != ftdi_write_data(jtag->ftdi, cmd, cmdlen)) {
    fprintf(stderr, "Failed to send high gpio command\n");
    return false;
  }

  cmd[0] = TCK_DIVISOR;
  cmd[1] = divisor & 0xff;
  cmd[2] = (divisor >> 8) & 0xff;
  if (cmdlen != ftdi_write_data(jtag->ftdi, cmd, cmdlen)) {
    fprintf(stderr, "Failed to send clock divisor command\n");
    return false;
  }

  cmd[0] = LOOPBACK_END;
  cmdlen = 1;
  if (cmdlen != ftdi_write_data(jtag->ftdi, cmd, 1)) {
    fprintf(stderr, "Failed to send loopback command\n");
    return false;
  }

  return true;
}

bool jtag_set_freq(struct jtag_ctx *jtag, double freq) {
  unsigned char cmd[3];
  int cmdlen = sizeof(cmd);
  int divisor = 30.0 / (freq / 1000000.0) - 1.0;

  if (!jtag->active) {
    fprintf(stderr,
            "Jtag must be connected and initialized before setting freq!\n");
    return false;
  }

  cmd[0] = TCK_DIVISOR;
  cmd[1] = divisor & 0xff;
  cmd[2] = (divisor >> 8) & 0xff;
  if (cmdlen != ftdi_write_data(jtag->ftdi, cmd, cmdlen)) {
    fprintf(stderr, "Failed to send freq clock divisor command\n");
    return false;
  }

  return true;
}

bool jtag_navigate_to_state(struct jtag_ctx *jtag, enum jtag_fsm_state init,
                            enum jtag_fsm_state dest) {
  unsigned char cmd[3];
  int cmdlen = sizeof(cmd);

  struct jtag_fsm_transitions transitions = get_transitions(init, dest);
  /*
   fprintf(stderr, "state1: %s, state2: %s, moves: %d, tms: %d\n",
   get_state_name(init), get_state_name(dest), transitions.moves,
   transitions.tms);
   */
  if (transitions.moves > 0) {
    if (transitions.moves < 8) {
      cmd[0] = 0x4B;
      cmd[1] = transitions.moves - 1;
      cmd[2] = 0x7f & transitions.tms;
      if (cmdlen != ftdi_write_data(jtag->ftdi, cmd, cmdlen)) {
        return false;
      }
    } else {
      fprintf(stdout, "Transition of 8 moves!\n");
      cmd[0] = 0x4B;
      cmd[1] = 6;
      cmd[2] = 0x7f & transitions.tms;
      if (cmdlen != ftdi_write_data(jtag->ftdi, cmd, cmdlen)) {
        return false;
      }
      cmd[0] = 0x4B;
      cmd[1] = transitions.moves - 8;
      cmd[2] = 0x7f & (transitions.tms >> 7);
      if (cmdlen != ftdi_write_data(jtag->ftdi, cmd, cmdlen)) {
        return false;
      }
    }
  }
  return true;
}

bool jtag_shift_data(struct jtag_ctx *jtag, unsigned int bits, char *tdi,
                     char *tdo, char *mask) {
  unsigned char cmd[3];
  int cmdlen = sizeof(cmd);
  unsigned int req_bytes = bits / 8 + (bits % 8 > 0);
  unsigned int req_hex = bits / 4 + (bits % 4 > 0);

  unsigned char *tdo_buf = NULL;
  unsigned int tdo_bytes = 0;

  if (strlen(tdi) < req_hex)
    return false;

  bool read = (tdo != NULL) && (strlen(tdo) > 0);
  if (read) {
    if (strlen(tdo) < req_hex) {
      return false;
    }
    if (mask != NULL && strlen(mask) < req_hex) {
      return false;
    }
  }

  if (!jtag_sync_mpsse(jtag))
    return false;

  if (bits < 9) {
    int data = strtol(tdi, NULL, 16);
    cmd[0] = read ? 0x3B : 0x1B;
    cmd[1] = bits - 2;
    cmd[2] = data & 0xff;
    if (cmdlen != ftdi_write_data(jtag->ftdi, cmd, cmdlen)) {
      return false;
    }

    unsigned char last_bit = (data >> ((bits - 1) % 8)) & 0x01;

    cmd[0] = read ? 0x6E : 0x4E;
    cmd[1] = 0x00;
    cmd[2] = 0x03 | (last_bit << 7);
    if (cmdlen != ftdi_write_data(jtag->ftdi, cmd, cmdlen)) {
      return false;
    }

    if (read) {
      tdo_buf = calloc(1, 3);
      tdo_bytes = ftdi_read_data(jtag->ftdi, cmd, 2);
      if (tdo_bytes != 2) {
        free(tdo_buf);
        fprintf(stderr, "Got %d TDO bytes where as only %d was expected\n",
                tdo_bytes, 2);
        return false;
      }

      tdo_buf[0] = cmd[0] >> (8 - (bits - 1));
      tdo_buf[0] |= cmd[1] >> (7 - (bits - 1));
    }
  } else {
    unsigned char *tdi_buf = calloc(1, req_bytes + 1);
    for (unsigned int i = 0; i < req_hex / 2; i++) {
      tdi_buf[i] = byte_from_hex_string(tdi, req_hex - 2 - i * 2, 2);
    }
    if ((req_hex & 1) != 0) {
      tdi_buf[req_hex / 2] = byte_from_hex_string(tdi, 0, 1);
    }

    unsigned int full_bytes = (bits - 1) / 8;
    unsigned int rem_bytes = full_bytes;
    unsigned int offset = 0;

    if (full_bytes > 65536 && read) {
      fprintf(stdout, "Large transfers with reads may not work!\n");
    }

    while (rem_bytes > 0) {
      unsigned int bct = rem_bytes > 65536 ? 65536 : rem_bytes;
      cmd[0] = read ? 0x39 : 0x19;
      cmd[1] = (bct - 1) & 0xff;
      cmd[2] = ((bct - 1) >> 8) & 0xff;

      if (cmdlen != ftdi_write_data(jtag->ftdi, cmd, cmdlen)) {
        return false;
      }
      if (bct != ftdi_write_data(jtag->ftdi, tdi_buf + offset, bct)) {
        return false;
      }

      rem_bytes -= bct;
      offset += bct;
    }

    unsigned int partial_bits = bits - 1 - (full_bytes * 8);
    if (full_bytes * 8 + 1 != bits) {
      cmd[0] = read ? 0x3B : 0x1B;
      cmd[1] = partial_bits - 1;
      cmd[2] = tdi_buf[req_bytes - 1] & 0xff;
      if (cmdlen != ftdi_write_data(jtag->ftdi, cmd, cmdlen)) {
        return false;
      }
    }

    unsigned char last_bit =
        (tdi_buf[req_bytes - 1] >> ((bits - 1) % 8)) & 0x01;
    cmd[0] = read ? 0x6E : 0x4E;
    cmd[1] = 0x00;
    cmd[2] = 0x03 | (last_bit << 7);
    if (cmdlen != ftdi_write_data(jtag->ftdi, cmd, cmdlen)) {
      return false;
    }

    if (read) {
      unsigned char *ibuf = calloc(1, req_bytes + 6);
      size_t bytes_to_read =
          full_bytes + ((full_bytes * 8 + 1 != bits) ? 2 : 1);
      if (bytes_to_read != ftdi_read_data(jtag->ftdi, ibuf, bytes_to_read)) {
        return false;
      }

      tdo_buf = calloc(1, req_bytes + 6);
      for (unsigned int i = 0; i < full_bytes; i++) {
        tdo_buf[tdo_bytes++] = ibuf[i];
      }

      if (full_bytes * 8 + 1 != bits) {
        tdo_buf[tdo_bytes] = ibuf[tdo_bytes] >> (8 - partial_bits);
        tdo_buf[tdo_bytes++] |= ibuf[bytes_to_read - 1] >> (7 - partial_bits);
      } else {
        tdo_buf[tdo_bytes++] = ibuf[bytes_to_read - 1] >> 7;
      }
      free(ibuf);
    }
  }

  bool ret = true;
  /*
  if (read) {
    // Read out the data from input buffer
    char *hextdo;
    int hextdoat = 0;
    hextdo = calloc(1, tdo_bytes * 2 + 1);

    for (int i = tdo_bytes - 1; i >= 0; i--) {
      sprintf(hextdo + hextdoat, "%02X", tdo_buf[i]);
      hextdoat += 2;
    }
    if ((strlen(hextdo) - 1) == strlen(mask)) {
      hextdoat = strlen(hextdo) - 1;
      hextdo[hextdoat] = 0x0;
    }

    fprintf(stdout, "hextdolen: %d\n", strlen(hextdo));
    if (!compare_hex_string(hextdo, tdo, mask)) {
      fprintf(stderr, "TDO didn't match expected string: \n");
      fprintf(stderr, "TDO:       %s\n", hextdo);
      fprintf(stderr, "EXPECTED:  %s\n", tdo);
      fprintf(stderr, "MASK:      %s\n", mask);
      ret = true;
    }
    free(hextdo);
  }
  */
  return ret;
}

bool jtag_send_clocks(struct jtag_ctx *jtag, unsigned long cycles) {
  unsigned char cmd[3];
  int cmdlen = sizeof(cmd);

  if (cycles / 8 > 65536) {
    if (!jtag_send_clocks(jtag, cycles - 65536 * 8)) {
      return false;
    }
    cycles = 65536 * 8;
  }
  cycles /= 8;

  cmd[0] = CLK_BYTES;
  cmd[1] = (cycles - 1) & 0xff;
  cmd[2] = ((cycles - 1) >> 8) & 0xff;
  if (cmdlen != ftdi_write_data(jtag->ftdi, cmd, cmdlen)) {
    return false;
  }

  return true;
}
