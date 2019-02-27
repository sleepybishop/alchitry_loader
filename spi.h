#ifndef SPI_H_
#define SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ftdi.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

struct spi_ctx {
  struct ftdi_context *ftdi;
  bool active;
  bool verbose;
};

struct spi_ctx *spi_new();
void spi_shutdown(struct spi_ctx *spi);
bool spi_initialize(struct spi_ctx *spi);
bool spi_erase_flash(struct spi_ctx *spi);
bool spi_write_bin(struct spi_ctx *spi, char *file);

#ifdef __cplusplus
}
#endif
#endif /* SPI_H_ */
