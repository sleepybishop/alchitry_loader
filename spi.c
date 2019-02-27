#include "spi.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define LATENCY_MS 2
#define CHUNK_SIZE 65535
#define USB_TIMEOUT 5000

static void check_rx(struct spi_ctx *);
static void error(struct spi_ctx *, int);
static void send_spi(struct spi_ctx *, uint8_t *data, int n);
static void xfer_spi(struct spi_ctx *, uint8_t *data, int n);
static uint8_t xfer_spi_bits(struct spi_ctx *, uint8_t data, int n);
static void set_gpio(struct spi_ctx *, int slavesel_b, int creset_b);
static int get_cdone(struct spi_ctx *);
static void flash_chip_select(struct spi_ctx *);
static void flash_chip_deselect(struct spi_ctx *);
static void flash_read_id(struct spi_ctx *);
static void flash_reset(struct spi_ctx *);
static void flash_power_up(struct spi_ctx *);
static void flash_power_down(struct spi_ctx *);
static uint8_t flash_read_status(struct spi_ctx *);
static void flash_write_enable(struct spi_ctx *);
static void flash_bulk_erase(struct spi_ctx *);
static void flash_64kB_sector_erase(struct spi_ctx *, int addr);
static void flash_prog(struct spi_ctx *, int addr, uint8_t *data, int n);
static void flash_wait(struct spi_ctx *);

static bool sync_mpsse(struct ftdi_context *ftdi);
static bool config_spi(struct ftdi_context *ftdi);

// ---------------------------------------------------------
// FLASH definitions
// ---------------------------------------------------------

/* Transfer Command bits */

/* All byte based commands consist of:
 * - Command byte
 * - Length lsb
 * - Length msb
 *
 * If data out is enabled the data follows after the above command bytes,
 * otherwise no additional data is needed.
 * - Data * n
 *
 * All bit based commands consist of:
 * - Command byte
 * - Length
 *
 * If data out is enabled a byte containing bitst to transfer follows.
 * Otherwise no additional data is needed. Only up to 8 bits can be transferred
 * per transaction when in bit mode.
 */

/* b 0000 0000
 *   |||| |||`- Data out negative enable. Update DO on negative clock edge.
 *   |||| ||`-- Bit count enable. When reset count represents bytes.
 *   |||| |`--- Data in negative enable. Latch DI on negative clock edge.
 *   |||| `---- LSB enable. When set clock data out LSB first.
 *   ||||
 *   |||`------ Data out enable
 *   ||`------- Data in enable
 *   |`-------- TMS mode enable
 *   `--------- Special command mode enable. See mpsse_cmd enum.
 */

/* Flash command definitions */
/* This command list is based on the Winbond W25Q128JV Datasheet */
enum flash_cmd {
  FC_WE = 0x06,      /* Write Enable */
  FC_SRWE = 0x50,    /* Volatile SR Write Enable */
  FC_WD = 0x04,      /* Write Disable */
  FC_RPD = 0xAB,     /* Release Power-Down, returns Device ID */
  FC_MFGID = 0x90,   /*  Read Manufacturer/Device ID */
  FC_JEDECID = 0x9F, /* Read JEDEC ID */
  FC_UID = 0x4B,     /* Read Unique ID */
  FC_RD = 0x03,      /* Read Data */
  FC_FR = 0x0B,      /* Fast Read */
  FC_PP = 0x02,      /* Page Program */
  FC_SE = 0x20,      /* Sector Erase 4kb */
  FC_BE32 = 0x52,    /* Block Erase 32kb */
  FC_BE64 = 0xD8,    /* Block Erase 64kb */
  FC_CE = 0xC7,      /* Chip Erase */
  FC_RSR1 = 0x05,    /* Read Status Register 1 */
  FC_WSR1 = 0x01,    /* Write Status Register 1 */
  FC_RSR2 = 0x35,    /* Read Status Register 2 */
  FC_WSR2 = 0x31,    /* Write Status Register 2 */
  FC_RSR3 = 0x15,    /* Read Status Register 3 */
  FC_WSR3 = 0x11,    /* Write Status Register 3 */
  FC_RSFDP = 0x5A,   /* Read SFDP Register */
  FC_ESR = 0x44,     /* Erase Security Register */
  FC_PSR = 0x42,     /* Program Security Register */
  FC_RSR = 0x48,     /* Read Security Register */
  FC_GBL = 0x7E,     /* Global Block Lock */
  FC_GBU = 0x98,     /* Global Block Unlock */
  FC_RBL = 0x3D,     /* Read Block Lock */
  FC_RPR = 0x3C,     /* Read Sector Protection Registers (adesto) */
  FC_IBL = 0x36,     /* Individual Block Lock */
  FC_IBU = 0x39,     /* Individual Block Unlock */
  FC_EPS = 0x75,     /* Erase / Program Suspend */
  FC_EPR = 0x7A,     /* Erase / Program Resume */
  FC_PD = 0xB9,      /* Power-down */
  FC_QPI = 0x38,     /* Enter QPI mode */
  FC_ERESET = 0x66,  /* Enable Reset */
  FC_RESET = 0x99,   /* Reset Device */
};

struct spi_ctx *spi_new(struct ftdi_context *ftdi) {
  struct spi_ctx *ctx = calloc(1, sizeof(struct spi_ctx));

  ctx->ftdi = ftdi;
  ctx->active = false;
  ctx->verbose = false;
  return ctx;
}

bool spi_initialize(struct spi_ctx *spi) {
  int status = 0;
  status |= ftdi_usb_reset(spi->ftdi);
  status |= ftdi_set_latency_timer(spi->ftdi, LATENCY_MS);
  status |= ftdi_write_data_set_chunksize(spi->ftdi, CHUNK_SIZE);
  status |= ftdi_read_data_set_chunksize(spi->ftdi, CHUNK_SIZE);
  status |= ftdi_set_bitmode(spi->ftdi, 0, BITMODE_RESET);
  status |= ftdi_set_bitmode(spi->ftdi, 0, BITMODE_MPSSE);

  spi->ftdi->usb_read_timeout = USB_TIMEOUT;
  spi->ftdi->usb_write_timeout = USB_TIMEOUT;

  if (status != 0) {
    fprintf(stderr, "Failed to set initial configuration!\n");
    return false;
  }

  usleep(100000);
  ftdi_usb_purge_buffers(spi->ftdi);

  if (!sync_mpsse(spi->ftdi)) {
    fprintf(stderr, "Failed to sync with MPSSE!\n");
    return false;
  }

  if (!config_spi(spi->ftdi)) {
    fprintf(stderr, "Failed to set SPI configuration!\n");
    return false;
  }

  spi->active = true;

  return true;
}

void spi_shutdown(struct spi_ctx *spi) {
  if (spi) {
    if (spi->active) {
      ftdi_set_bitmode(spi->ftdi, 0, BITMODE_RESET);
      ftdi_usb_close(spi->ftdi);
      ftdi_deinit(spi->ftdi);
    }
    free(spi);
    spi = NULL;
  }
}

bool sync_mpsse(struct ftdi_context *ftdi) {
  unsigned char cmd[2] = {0xaa, 0x0};
  int cmdlen = 1;

  if (cmdlen != ftdi_write_data(ftdi, cmd, cmdlen)) {
    fprintf(stderr, "Failed to send bad command\n");
  }

  int n = 0, r = 0;
  while (n < cmdlen) {
    r = ftdi_read_data(ftdi, cmd, cmdlen);
    if (r < 0)
      break;
    n += r;
  }
  ftdi_usb_purge_rx_buffer(ftdi);

  return n == cmdlen;
}

bool config_spi(struct ftdi_context *ftdi) {
  unsigned char cmd[3];
  int cmdlen = sizeof(cmd);

  // -----------------------------------------------------------
  // Configure the MPSSE settings for SPI
  // -----------------------------------------------------------
  cmd[0] = DIS_DIV_5;
  cmd[1] = DIS_ADAPTIVE;
  cmd[2] = DIS_3_PHASE;
  if (cmdlen != ftdi_write_data(ftdi, cmd, cmdlen)) {
    fprintf(stderr, "Failed to send speed command\n");
    return false;
  }

  // Set initial states of the MPSSE interface - low byte, both pin directions
  // and output values
  // Pin name Signal Direction Config Initial State Config
  // ADBUS0 SCK output 1 low 0
  // ADBUS1 MOSI output 1 low 0
  // ADBUS2 MISO input 0 low 0
  // ADBUS3 NC output 1 low 0
  // ADBUS4 SS output 1 low 0
  // ADBUS5 NC output 1 low 0
  // ADBUS6 CDONE input 0 low 0
  // ADBUS7 CRESET output 1 low 0
  cmd[0] = SET_BITS_LOW;
  cmd[1] = 0x00;
  cmd[2] = 0xBB;
  if (cmdlen != ftdi_write_data(ftdi, cmd, cmdlen)) {
    fprintf(stderr, "Failed to send low gpio command\n");
    return false;
  }

  // Set initial states of the MPSSE interface - high byte, both pin directions
  // and output values
  // Pin name Signal Direction Config Initial State Config
  // ACBUS0 GPIOH0 input 0 0
  // ACBUS1 GPIOH1 input 0 0
  // ACBUS2 GPIOH2 input 0 0
  // ACBUS3 GPIOH3 input 0 0
  // ACBUS4 GPIOH4 input 0 0
  // ACBUS5 GPIOH5 input 0 0
  // ACBUS6 GPIOH6 input 0 0
  // ACBUS7 GPIOH7 input 0 0
  cmd[0] = SET_BITS_HIGH;
  cmd[1] = 0x00;
  cmd[2] = 0x00;
  if (cmdlen != ftdi_write_data(ftdi, cmd, cmdlen)) {
    fprintf(stderr, "Failed to send high gpio command\n");
    return false;
  }

  cmd[0] = TCK_DIVISOR;
  cmd[1] = 0x0;
  cmd[2] = 0x0;
  if (cmdlen != ftdi_write_data(ftdi, cmd, cmdlen)) {
    fprintf(stderr, "Failed to send clock divisor command\n");
    return false;
  }

  if (cmdlen != ftdi_write_data(ftdi, cmd, cmdlen)) {
    fprintf(stderr, "Failed to send clock divisor command\n");
    return false;
  }

  cmd[0] = LOOPBACK_END;
  cmdlen = 1;
  if (cmdlen != ftdi_write_data(ftdi, cmd, 1)) {
    fprintf(stderr, "Failed to send loopback command\n");
    return false;
  }

  return true;
}

void check_rx(struct spi_ctx *spi) {
  unsigned char cmd[1];
  int cmdlen = 1;

  while (1) {
    cmdlen = ftdi_read_data(spi->ftdi, cmd, 1);
    if (cmdlen != 1)
      break;
    fprintf(stderr, "Unexpected rx byte: %x\n", cmd[0]);
  }
}

void error(struct spi_ctx *spi, int status) {
  check_rx(spi);
  fprintf(stderr, "ABORT.\n");
  if (spi->active)
    spi_shutdown(spi);
  exit(status);
}

void send_spi(struct spi_ctx *spi, uint8_t *data, int n) {
  unsigned char cmd[3];
  int cmdlen = 3;

  if (n < 1)
    return;

  // Output only, update data on negative clock edge.
  cmd[0] = MPSSE_DO_WRITE | MPSSE_WRITE_NEG;
  cmd[1] = (n - 1) & 0xff;
  cmd[2] = ((n - 1) >> 8) & 0xff;
  if (cmdlen != ftdi_write_data(spi->ftdi, cmd, cmdlen)) {
    fprintf(stderr, "Write error!\n");
    error(spi, 2);
  }

  int len = ftdi_write_data(spi->ftdi, data, n);
  if (n != len) {
    fprintf(stderr, "Write error (chunk, rc=%d, expected %d).\n", len, n);
    error(spi, 2);
  }
}

void xfer_spi(struct spi_ctx *spi, uint8_t *data, int n) {
  unsigned char cmd[3], *p = data;
  int cmdlen = 3, len = 0, rem = n;

  if (n < 1)
    return;

  // Input and output, update data on negative edge read on positive.
  cmd[0] = MPSSE_DO_READ | MPSSE_DO_WRITE | MPSSE_WRITE_NEG;
  cmd[1] = (n - 1) & 0xff;
  cmd[2] = ((n - 1) >> 8) & 0xff;
  if (cmdlen != ftdi_write_data(spi->ftdi, cmd, cmdlen)) {
    fprintf(stderr, "Write error!\n");
    error(spi, 2);
  }

  len = ftdi_write_data(spi->ftdi, data, n);
  if (n != len) {
    fprintf(stderr, "Write error (chunk, rc=%d, expected %d).\n", len, n);
    error(spi, 2);
  }

  while (rem > 0) {
    len = ftdi_read_data(spi->ftdi, p, rem);
    if (len < 0) {
      fprintf(stderr, "Read error (chunk, rc=%d, expected %d).\n", len, n);
      error(spi, 2);
    }
    p += len;
    rem -= len;
  }
}

uint8_t xfer_spi_bits(struct spi_ctx *spi, uint8_t data, int n) {
  unsigned char cmd[3];
  int cmdlen = 3;

  if (n < 1)
    return 0;

  // Input and output, update data on negative edge read on positive, bits.
  cmd[0] = MPSSE_DO_READ | MPSSE_DO_WRITE | MPSSE_WRITE_NEG | MPSSE_BITMODE;
  cmd[1] = (n - 1);
  cmd[2] = data;
  if (cmdlen != ftdi_write_data(spi->ftdi, cmd, cmdlen)) {
    fprintf(stderr, "Write error!\n");
    error(spi, 2);
  }

  cmdlen = ftdi_read_data(spi->ftdi, cmd, 1);
  if (cmdlen != 1) {
    fprintf(stderr, "Read error.\n");
    error(spi, 2);
  }

  return cmd[0];
}

void set_gpio(struct spi_ctx *spi, int slavesel_b, int creset_b) {
  unsigned char cmd[3];
  int cmdlen = 3;
  uint8_t gpio = 0;

  if (slavesel_b) {
    // ADBUS4 (GPIOL0)
    gpio |= 0x10;
  }

  if (creset_b) {
    // ADBUS7 (GPIOL3)
    gpio |= 0x80;
  }

  cmd[0] = SET_BITS_LOW;
  cmd[1] = (gpio); // Value
  cmd[2] = (0x93); // Direction
  if (cmdlen != ftdi_write_data(spi->ftdi, cmd, cmdlen)) {
    fprintf(stderr, "Write error!\n");
    error(spi, 2);
  }
}

int get_cdone(struct spi_ctx *spi) {
  unsigned char cmd[1];
  int cmdlen = 1;

  cmd[0] = GET_BITS_LOW;
  if (cmdlen != ftdi_write_data(spi->ftdi, cmd, cmdlen)) {
    fprintf(stderr, "Write error!\n");
    error(spi, 2);
  }

  cmdlen = ftdi_read_data(spi->ftdi, cmd, 1);
  if (cmdlen != 1) {
    fprintf(stderr, "Read error.\n");
    error(spi, 2);
  }

  // ADBUS6 (GPIOL2)
  return (cmd[0] & 0x40) != 0;
}

// ---------------------------------------------------------
// FLASH function implementations
// ---------------------------------------------------------

// FLASH chip select assert
// should only happen while FPGA reset is asserted
void flash_chip_select(struct spi_ctx *spi) { set_gpio(spi, 0, 0); }

// FLASH chip select deassert
void flash_chip_deselect(struct spi_ctx *spi) { set_gpio(spi, 1, 0); }

void flash_read_id(struct spi_ctx *spi) {
  /* JEDEC ID structure:
   * Byte No. | Data Type
   * ---------+----------
   *        0 | FC_JEDECID Request Command
   *        1 | MFG ID
   *        2 | Dev ID 1
   *        3 | Dev ID 2
   *        4 | Ext Dev Str Len
   */

  uint8_t data[260] = {FC_JEDECID};
  int len = 5; // command + 4 response bytes

  if (spi->verbose)
    fprintf(stdout, "read flash ID..\n");

  flash_chip_select(spi);

  // Write command and read first 4 bytes
  xfer_spi(spi, data, len);

  if (data[4] == 0xFF) {
    fprintf(stderr, "Extended Device String Length is 0xFF, "
                    "this is likely a read error. Ignoring...\n");
  } else {
    // Read extended JEDEC ID bytes
    if (data[4] != 0) {
      len += data[4];
      xfer_spi(spi, data + 5, len - 5);
    }
  }

  flash_chip_deselect(spi);

  if (spi->verbose) {
    fprintf(stdout, "flash ID:");
    for (int i = 1; i < len; i++)
      fprintf(stdout, " 0x%02X", data[i]);
    fprintf(stdout, "\n");
  }
}

void flash_reset(struct spi_ctx *spi) {
  flash_chip_select(spi);
  xfer_spi_bits(spi, 0xFF, 8);
  flash_chip_deselect(spi);

  flash_chip_select(spi);
  xfer_spi_bits(spi, 0xFF, 2);
  flash_chip_deselect(spi);
}

void flash_power_up(struct spi_ctx *spi) {
  uint8_t data_rpd[1] = {FC_RPD};
  flash_chip_select(spi);
  xfer_spi(spi, data_rpd, 1);
  flash_chip_deselect(spi);
}

void flash_power_down(struct spi_ctx *spi) {
  uint8_t data[1] = {FC_PD};
  flash_chip_select(spi);
  xfer_spi(spi, data, 1);
  flash_chip_deselect(spi);
}

uint8_t flash_read_status(struct spi_ctx *spi) {
  uint8_t data[2] = {FC_RSR1};

  flash_chip_select(spi);
  xfer_spi(spi, data, 2);
  flash_chip_deselect(spi);

  if (spi->verbose) {
    fprintf(stdout, "SR1: 0x%02X\n", data[1]);
    fprintf(stdout, " - SPRL: %s\n",
            ((data[1] & (1 << 7)) == 0) ? "unlocked" : "locked");
    fprintf(stdout, " -  SPM: %s\n",
            ((data[1] & (1 << 6)) == 0) ? "Byte/Page Prog Mode"
                                        : "Sequential Prog Mode");
    fprintf(stdout, " -  EPE: %s\n",
            ((data[1] & (1 << 5)) == 0) ? "Erase/Prog success"
                                        : "Erase/Prog error");
    fprintf(stdout, "-  SPM: %s\n",
            ((data[1] & (1 << 4)) == 0) ? "~WP asserted" : "~WP deasserted");
    fprintf(stdout, " -  SWP: ");
    switch ((data[1] >> 2) & 0x3) {
    case 0:
      fprintf(stdout, "All sectors unprotected\n");
      break;
    case 1:
      fprintf(stdout, "Some sectors protected\n");
      break;
    case 2:
      fprintf(stdout, "Reserved (xxxx 10xx)\n");
      break;
    case 3:
      fprintf(stdout, "All sectors protected\n");
      break;
    }
    fprintf(stdout, " -  WEL: %s\n",
            ((data[1] & (1 << 1)) == 0) ? "Not write enabled"
                                        : "Write enabled");
    fprintf(stdout, " - ~RDY: %s\n",
            ((data[1] & (1 << 0)) == 0) ? "Ready" : "Busy");
  }

  usleep(1000);
  return data[1];
}

void flash_write_enable(struct spi_ctx *spi) {
  if (spi->verbose) {
    fprintf(stdout, "status before enable:\n");
    flash_read_status(spi);
  }

  if (spi->verbose)
    fprintf(stdout, "write enable..\n");

  uint8_t data[1] = {FC_WE};
  flash_chip_select(spi);
  xfer_spi(spi, data, 1);
  flash_chip_deselect(spi);

  if (spi->verbose) {
    fprintf(stdout, "status after enable:\n");
    flash_read_status(spi);
  }
}

void flash_bulk_erase(struct spi_ctx *spi) {
  if (spi->verbose)
    fprintf(stdout, "bulk erase..\n");

  uint8_t data[1] = {FC_CE};
  flash_chip_select(spi);
  xfer_spi(spi, data, 1);
  flash_chip_deselect(spi);
}

void flash_64kB_sector_erase(struct spi_ctx *spi, int addr) {
  if (spi->verbose)
    fprintf(stdout, "erase 64kB sector at 0x%06X..\n", addr);

  uint8_t command[4] = {FC_BE64, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8),
                        (uint8_t)addr};

  flash_chip_select(spi);
  send_spi(spi, command, 4);
  flash_chip_deselect(spi);
}

void flash_prog(struct spi_ctx *spi, int addr, uint8_t *data, int n) {
  if (spi->verbose)
    fprintf(stdout, "prog 0x%06X +0x%03X..\n", addr, n);

  uint8_t command[4] = {FC_PP, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8),
                        (uint8_t)addr};

  flash_chip_select(spi);
  send_spi(spi, command, 4);
  send_spi(spi, data, n);
  flash_chip_deselect(spi);

  if (spi->verbose)
    for (int i = 0; i < n; i++)
      fprintf(stderr, "%02x%c", data[i],
              i == n - 1 || i % 32 == 31 ? '\n' : ' ');
}

void flash_wait(struct spi_ctx *spi) {
  if (spi->verbose)
    fprintf(stderr, "waiting..");

  int count = 0;
  while (1) {
    uint8_t data[2] = {FC_RSR1};

    flash_chip_select(spi);
    xfer_spi(spi, data, 2);
    flash_chip_deselect(spi);

    if ((data[1] & 0x01) == 0) {
      if (count < 2) {
        count++;
        if (spi->verbose) {
          fprintf(stderr, "r");
          fflush(stderr);
        }
      } else {
        if (spi->verbose) {
          fprintf(stderr, "R");
          fflush(stderr);
        }
        break;
      }
    } else {
      if (spi->verbose) {
        fprintf(stderr, ".");
        fflush(stderr);
      }
      count = 0;
    }

    usleep(1000);
  }

  if (spi->verbose)
    fprintf(stderr, "\n");
}

bool spi_erase_flash(struct spi_ctx *spi) {
  fprintf(stdout, "Resetting...\n");

  flash_chip_deselect(spi);
  usleep(250000);

  if (spi->verbose) {
    fprintf(stdout, "cdone: %s\n", get_cdone(spi) ? "high" : "low");
  }

  flash_reset(spi);
  flash_power_up(spi);

  flash_read_id(spi);

  flash_write_enable(spi);
  flash_bulk_erase(spi);
  flash_wait(spi);

  // ---------------------------------------------------------
  // Reset
  // ---------------------------------------------------------

  flash_power_down(spi);

  set_gpio(spi, 1, 1);
  usleep(250000);

  if (spi->verbose) {
    fprintf(stdout, "cdone: %s\n", get_cdone(spi) ? "high" : "low");
  }

  return true;
}

bool spi_write_bin(struct spi_ctx *spi, char *filename) {
  int rw_offset = 0;

  FILE *f = NULL;
  long file_size = -1;

  f = fopen(filename, "rb");

  if (f == NULL) {
    fprintf(stderr, "Can't open '%s' for reading: ", filename);
    return false;
  }

  if (fseek(f, 0L, SEEK_END) != -1) {
    file_size = ftell(f);
    if (file_size == -1) {
      fprintf(stderr, "%s: ftell: ", filename);
      return false;
    }
    if (fseek(f, 0L, SEEK_SET) == -1) {
      fprintf(stderr, "%s: fseek: ", filename);
      return false;
    }
  }

  fprintf(stdout, "Resetting...\n");

  flash_chip_deselect(spi);
  usleep(250000);

  if (spi->verbose) {
    fprintf(stdout, "cdone: %s\n", get_cdone(spi) ? "high" : "low");
  }

  flash_reset(spi);
  flash_power_up(spi);

  flash_read_id(spi);

  int begin_addr = rw_offset & ~0xffff;
  int end_addr = (rw_offset + file_size + 0xffff) & ~0xffff;

  for (int addr = begin_addr; addr < end_addr; addr += 0x10000) {
    flash_write_enable(spi);
    flash_64kB_sector_erase(spi, addr);
    if (spi->verbose) {
      fprintf(stderr, "Status after block erase:\n");
      flash_read_status(spi);
    }
    flash_wait(spi);
  }

  fprintf(stdout, "Programming...");
  for (int rc, addr = 0; true; addr += rc) {
    uint8_t buffer[256];
    int page_size = 256 - (rw_offset + addr) % 256;
    rc = fread(buffer, 1, page_size, f);
    if (rc <= 0)
      break;
    flash_write_enable(spi);
    flash_prog(spi, rw_offset + addr, buffer, rc);
    flash_wait(spi);
  }

  fprintf(stdout, "Done.\n");

  // seek to the beginning for second pass
  fseek(f, 0, SEEK_SET);

  // ---------------------------------------------------------
  // Reset
  // ---------------------------------------------------------

  flash_power_down(spi);

  set_gpio(spi, 1, 1);
  usleep(250000);

  if (spi->verbose) {
    fprintf(stdout, "cdone: %s\n", get_cdone(spi) ? "high" : "low");
  }
  fprintf(stdout, "Done.\n");

  if (f != NULL && f != stdin && f != stdout)
    fclose(f);
  return true;
}
