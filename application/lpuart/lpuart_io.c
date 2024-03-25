#include "lpuart_io.h"
#include "fsl_reset.h"

static bool init_lpuart2(struct lpuart_io *handle);

bool init_lpuart(struct lpuart_io *handle, LPUART_Type *inst, uint32_t baud)
{       
    LPUART_GetDefaultConfig(&handle->config);
    handle->config.baudRate_Bps = baud;
    handle->config.enableTx     = true;
    handle->config.enableRx     = true;
    handle->config.enableRxRTS  = true;
    handle->config.enableTxCTS  = true;

    if(inst == LPUART2_BASE) {
        handle->instance = inst;
        if(!init_lpuart2(handle))
            goto error_handler;
    } else {
        handle->instance = NULL;
        goto error_handler;
    }

    return true;

error_handler:
    deinit_lpuart(handle);
    return false;
}

size_t read_lpuart(struct lpuart_io *handle, uint8_t *buf, size_t size)
{
    size_t read_bytes = 0;

    while(!(kLPUART_RxFifoEmptyFlag & LPUART_GetStatusFlags(handle->instance))) {
        if(read_bytes >= size)
            break;
        buf[read_bytes++] = (uint8_t)(handle->instance->DATA);
    }
    
    return read_bytes;
}

bool write_lpuart(struct lpuart_io *handle, uint8_t *buf, size_t size)
{
    return LPUART_WriteBlocking(handle->instance, buf, size) == kStatus_Success;
}

static bool init_lpuart2(struct lpuart_io *handle)
{    
    CLOCK_SetIpSrc(kCLOCK_Lpuart2, kCLOCK_Pcc2BusIpSrcFusionDspBus);

    RESET_PeripheralReset(kRESET_Lpuart2);

    return LPUART_Init(handle->instance, &handle->config, 
            CLOCK_GetFreq(kCLOCK_FusionDspBusClk)) == kStatus_Success;
}

void deinit_lpuart(struct lpuart_io *handle)
{
    if(handle->instance != NULL)
        LPUART_Deinit(handle->instance);
}