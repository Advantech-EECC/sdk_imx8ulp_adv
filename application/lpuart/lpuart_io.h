#ifndef _LPUART_IO_H_
#define _LPUART_IO_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "fsl_lpuart.h"
#include "MIMX8UD7_cm33.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"

struct lpuart_io {
    LPUART_Type *instance;
    lpuart_config_t config;
    StreamBufferHandle_t rx_buffer;
};

bool init_lpuart(struct lpuart_io *handle, LPUART_Type *inst, uint32_t baud);
uint32_t read_lpuart(struct lpuart_io *handle, uint8_t *buf, uint32_t size);
uint32_t write_lpuart(struct lpuart_io *handle, uint8_t *buf, uint32_t size);
void deinit_lpuart(struct lpuart_io *handle);
size_t read_lpuart_interrupt(struct lpuart_io *handle, uint8_t *buf, size_t size);

#endif
