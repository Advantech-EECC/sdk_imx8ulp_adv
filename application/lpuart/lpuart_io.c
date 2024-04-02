#include "lpuart_io.h"
#include "fsl_reset.h"

static struct lpuart_io *ext_handle = NULL;

static bool init_lpuart2(struct lpuart_io *handle);

bool init_lpuart(struct lpuart_io *handle, LPUART_Type *inst, uint32_t baud)
{       
    LPUART_GetDefaultConfig(&handle->config);
    handle->config.baudRate_Bps = baud;
    handle->config.enableTx     = true;
    handle->config.enableRx     = true;
    handle->config.enableRxRTS  = true;
    handle->config.enableTxCTS  = true;

    if(inst == (LPUART_Type *)LPUART2_BASE) {
        handle->instance = inst;
        if(!init_lpuart2(handle))
            goto error_handler;
    } else {
        handle->instance = NULL;
        goto error_handler;
    }

    handle->rx_buffer = xStreamBufferCreate(512, 1);

    ext_handle = handle;

    LPUART_EnableInterrupts(handle->instance, kLPUART_RxDataRegFullInterruptEnable);
    EnableIRQ(LPUART2_IRQn);

    return true;

error_handler:
    deinit_lpuart(handle);
    return false;
}

uint32_t read_lpuart(struct lpuart_io *handle, uint8_t *buf, uint32_t size)
{
    uint32_t read_bytes = 0;

    if(0U == (handle->instance->STAT & LPUART_STAT_RDRF_MASK))
        return read_bytes;

    if(kLPUART_RxOverrunFlag & LPUART_GetStatusFlags(handle->instance))
        LPUART_ClearStatusFlags(handle->instance, kLPUART_RxOverrunFlag);

    while((handle->instance->STAT & LPUART_STAT_RDRF_MASK)) {
        if(read_bytes >= size)
            break;
        buf[read_bytes++] = (uint8_t)(handle->instance->DATA);
    }

    return read_bytes;
}

size_t read_lpuart_interrupt(struct lpuart_io *handle, uint8_t *buf, size_t size)
{
    size_t read_bytes = 0;
    size_t bytes_avl = xStreamBufferBytesAvailable(handle->rx_buffer);
    if(bytes_avl > size)
        bytes_avl = size;

    if(bytes_avl)
        read_bytes = xStreamBufferReceive(handle->rx_buffer, (void *)buf, bytes_avl, 0);
    
    return read_bytes;
}

uint32_t write_lpuart(struct lpuart_io *handle, uint8_t *buf, uint32_t size)
{
    const uint32_t tx_fifo_max = 8;
    uint32_t c = LPUART_GetTxFifoCount(LPUART2);
    uint32_t available = c < tx_fifo_max ? tx_fifo_max - c : 0;
    uint32_t written = 0;

    if (size > available)
        size = available;

    for (; written < size; written++)
       LPUART_WriteByte(LPUART2, buf[written]);

    return written;
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

void LPUART2_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(ext_handle != NULL) {
        uint8_t data;
        /* If new data arrived. */
        if ((kLPUART_RxDataRegFullFlag)&LPUART_GetStatusFlags(ext_handle->instance))
        {
            data = LPUART_ReadByte(ext_handle->instance);
            xStreamBufferSendFromISR(ext_handle->rx_buffer, &data, 1, &xHigherPriorityTaskWoken);
        }
    }
    SDK_ISR_EXIT_BARRIER;
}
