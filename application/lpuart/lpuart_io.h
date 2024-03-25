#ifndef _LPUART_IO_H_
#define _LPUART_IO_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "fsl_lpuart.h"
#include "MIMX8UD7_cm33.h"

struct lpuart_io {
    LPUART_Type *instance;
    lpuart_config_t config;
};

bool init_lpuart(struct lpuart_io *handle, LPUART_Type *inst, uint32_t baud);
size_t read_lpuart(struct lpuart_io *handle, uint8_t *buf, size_t size);
bool write_lpuart(struct lpuart_io *handle, uint8_t *buf, size_t size);
void deinit_lpuart(struct lpuart_io *handle);

#endif