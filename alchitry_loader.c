#include <ctype.h>
#include <ftdi.h>
#include <getopt.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "jtag.h"
#include "jtag_fsm.h"
#include "loader.h"
//#include "spi.h"

#define BOARD_ERROR -2
#define BOARD_UNKNOWN -1
#define BOARD_AU 0
#define BOARD_CU 1

#define VID 0x0403
#define PID 0x6010

/*
 * VID:     0x0403
 * PID:     0x6010
 * Release: 0x0700
 * Bus Powered: 500 mA
 * Manufacturer: Alchitry
 * Product:      Alchitry Au
 * Serial:       FT3KRFFN
 * Checksum      : c909
 * Attached EEPROM: 93x56
 * PNP: 1
 * Channel A has Mode FIFO
 * Channel B has Mode UART VCP
 * AL has 4 mA drive
 * AH has 4 mA drive
 * BL has 4 mA drive
 * BH has 4 mA drive
 */
char ManufacturerBuf[32];
char ManufacturerIdBuf[16];
char DescriptionBuf[64];
char SerialNumberBuf[16];

void erase(struct ftdi_context *ftdi) {
  fprintf(stdout, "Erasing... ");

  if (0 > ftdi_erase_eeprom(ftdi)) {
    fprintf(stdout, "Erase failed: %s\n", ftdi_get_error_string(ftdi));
    return;
  }
  fprintf(stdout, "Done.\n");
}

bool program_device(struct ftdi_context *ftdi, unsigned int device_num) {

  if (0 >
      ftdi_set_interface(ftdi, (device_num == 0) ? INTERFACE_A : INTERFACE_B)) {
    fprintf(stdout, "Failed to select interface: %s\n",
            ftdi_get_error_string(ftdi));
    return false;
  }

  // try 0000:0000 when blank
  if (0 > ftdi_usb_open(ftdi, VID, PID)) {
    fprintf(stdout, "Failed to open usb device: %s\n",
            ftdi_get_error_string(ftdi));
    return false;
  }

  erase(ftdi);

  ftdi_eeprom_initdefaults(ftdi, "Alchitry", "Alchitry Au", "FT3KRFFN");
  ftdi_set_eeprom_value(ftdi, VENDOR_ID, 0x0403);
  ftdi_set_eeprom_value(ftdi, PRODUCT_ID, 0x6010);
  ftdi_set_eeprom_value(ftdi, RELEASE_NUMBER, 0x700);
  ftdi_set_eeprom_value(ftdi, MAX_POWER, 500);
  ftdi_set_eeprom_value(ftdi, CHIP_SIZE, 256);
  ftdi_set_eeprom_value(ftdi, CHIP_TYPE, 86);
  ftdi_set_eeprom_value(ftdi, CHANNEL_A_TYPE, CHANNEL_IS_FIFO);
  ftdi_set_eeprom_value(ftdi, CHANNEL_B_TYPE, CHANNEL_IS_UART);
  ftdi_set_eeprom_value(ftdi, CHANNEL_B_DRIVER, DRIVER_VCP);

  fprintf(stdout, "Programming... ");
  ftdi_eeprom_build(ftdi);
  if (0 > ftdi_write_eeprom(ftdi)) {
    fprintf(stdout, "writing to EEPROM failed: %s\n",
            ftdi_get_error_string(ftdi));
    return false;
  }
  fprintf(stdout, "Checking EEPROM...\n");
  if (0 > ftdi_read_eeprom(ftdi)) {
    fprintf(stdout, "Reading EEPROM failed: %s\n", ftdi_get_error_string(ftdi));
    return false;
  }
  ftdi_eeprom_decode(ftdi, 1);

  fprintf(stdout, "Done.\n");
  return true;
}

void print_devices(struct ftdi_context *ftdi) {
  int i = 0;
  char mfg[32], desc[64], ser[16];
  struct ftdi_device_list *devlist = NULL, *dev;

  if (ftdi_usb_find_all(ftdi, &devlist, VID, PID) < 0) {
    fprintf(stderr, "Error getting device list!\n");
  }

  dev = devlist;
  if (!dev) {
    fprintf(stdout, "No devices found!\n");
    return;
  }

  while (dev) {
    ftdi_usb_get_strings(ftdi, dev->dev, mfg, sizeof(mfg), desc, sizeof(desc),
                         ser, sizeof(ser));
    fprintf(stdout, "%d: %s|%s|%s\n", i, mfg, desc, ser);
    i++;
    dev = dev->next;
  }

  ftdi_list_free(&devlist);
}

int get_device_type(struct ftdi_context *ftdi, unsigned int device_num) {
  int i = 0, board = BOARD_ERROR;
  char mfg[32], desc[64], ser[16];
  struct ftdi_device_list *devlist = NULL, *dev;

  if (ftdi_usb_find_all(ftdi, &devlist, VID, PID) < 0) {
    fprintf(stderr, "Error getting device list!\n");
  }

  dev = devlist;
  if (!dev) {
    fprintf(stdout, "No devices found!\n");
    return board;
  }

  while (dev) {
    if (i == device_num) {
      ftdi_usb_get_strings(ftdi, dev->dev, mfg, sizeof(mfg), desc, sizeof(desc),
                           ser, sizeof(ser));
      if (strcmp(desc, "Alchitry Au") == 0) {
        board = BOARD_AU;
      } else if (strcmp(desc, "Alchitry Cu") == 0) {
        board = BOARD_CU;
      } else {
        board = BOARD_UNKNOWN;
      }
    }
    i++;
    dev = dev->next;
  }
  ftdi_list_free(&devlist);

  return board;
}

void print_usage() {
  fprintf(stdout, "Usage: \"loader arguments\"\n\n");

  fprintf(stdout, "Arguments:\n");
  fprintf(stdout, "  -e : erase FPGA flash\n");
  fprintf(stdout, "  -l : list detected boards\n");
  fprintf(stdout, "  -h : print this help message\n");
  fprintf(stdout, "  -f config.bin : write FPGA flash\n");
  fprintf(stdout, "  -r config.bin : write FPGA RAM\n");
  fprintf(stdout, "  -u config.data : write FTDI eeprom\n");
  fprintf(stdout, "  -b n : select board \"n\" (defaults to 0)\n");
  fprintf(stdout, "  -p loader.bin : Au bridge bin\n");
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    print_usage();
    return 1;
  }

  int i = 0;
  bool fpga_flash = false, fpga_ram = false, eeprom = false;
  bool erase = false, list = false, print = false;
  bool bridge_provided = false;
  char *fpga_bin_flash, *fpga_bin_ram, *au_bridge_bin;
  int device_num = 0;

  struct ftdi_context *ftdi;
  if ((ftdi = ftdi_new()) == 0) {
    fprintf(stderr, "Failed to allocate ftdi structure :%s \n",
            ftdi_get_error_string(ftdi));
    return EXIT_FAILURE;
  }

  while ((i = getopt(argc, argv, "elhf:r:u:b:p:")) != -1) {
    switch (i) {
    case 'e':
      erase = true;
      break;
    case 'l':
      list = true;
      break;
    case 'h':
      print = true;
      break;
    case 'f':
      fpga_flash = true;
      fpga_bin_flash = optarg;
      break;
    case 'r':
      fpga_ram = true;
      fpga_bin_ram = optarg;
      break;
    case 'u':
      eeprom = true;
      break;
    case 'b':
      device_num = strtol(optarg, NULL, 10);
      break;
    case 'p':
      bridge_provided = true;
      au_bridge_bin = optarg;
      break;
    default:
      print_usage();
    }
  }

  if (print)
    print_usage();

  if (list)
    print_devices(ftdi);

  if (eeprom)
    program_device(ftdi, device_num);

  if (erase || fpga_flash || fpga_ram) {
    int board_type = get_device_type(ftdi, device_num);
    if (board_type == BOARD_AU) {
      if (bridge_provided == false && (erase || fpga_flash)) {
        fprintf(stderr, "No Au bridge bin provided!\n");
        return 2;
      }
      ftdi_usb_open(ftdi, VID, PID);
      struct jtag_ctx *jtag = jtag_new(ftdi);
      if (jtag_initialize(jtag) == false) {
        fprintf(stderr, "Failed to initialize JTAG!\n");
        return 2;
      }
      struct loader_ctx *loader = loader_new(jtag);

      /*
      if (!loader_check_IDCODE(loader)) {
        fprintf(stderr, "IDCODE check failed!\n");
        return 2;
      }
      */

      if (erase) {
        if (!loader_erase_flash(loader, au_bridge_bin)) {
          fprintf(stderr, "Failed to erase flash!\n");
          return 2;
        }
      }

      if (fpga_flash) {
        if (!loader_write_bin(loader, fpga_bin_flash, true, au_bridge_bin)) {
          fprintf(stderr, "Failed to write FPGA flash!\n");
          return 2;
        }
      }

      if (fpga_ram) {
        if (!loader_write_bin(loader, fpga_bin_ram, false, NULL)) {
          fprintf(stderr, "Failed to write FPGA RAM!\n");
          return 2;
        }
      }

      jtag_shutdown(jtag);
    } else if (board_type == BOARD_CU) {
      if (fpga_ram) {
        fprintf(stderr, "Alchitry Cu doesn't support RAM only programming!\n");
      }
      return 1;
    } else {
      fprintf(stderr, "Unknown board type!\n");
      return 2;
    }
  }
  return 0;
}
