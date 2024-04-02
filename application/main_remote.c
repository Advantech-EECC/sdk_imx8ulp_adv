/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"

#include "fsl_reset.h"
#include "app_srtm.h"
#include "fsl_upower.h"
#include "fsl_fusion.h"
#include "fsl_iomuxc.h"
#include "lpuart_io.h"

extern void app_create_task(void);
extern void app_destroy_task(void);
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RPMSG_LITE_LINK_ID            (RL_PLATFORM_IMX8ULP_M33_A35_USER_LINK_ID)
#define RPMSG_LITE_SHMEM_BASE         (VDEV1_VRING_BASE)
#define RPMSG_LITE_NS_ANNOUNCE_STRING "rpmsg-virtual-tty-channel"
#define RPMSG_LITE_MASTER_IS_LINUX

#define APP_DEBUG_UART_BAUDRATE (115200U) /* Debug console baud rate. */
#define APP_TASK_STACK_SIZE (256)
#ifndef LOCAL_EPT_ADDR
#define LOCAL_EPT_ADDR (30)
#endif

/* Globals */

static uint8_t buf_m2u[512]; // mailbox -> uart buffer
static uint8_t buf_u2m[512]; // uart -> mailbox buffer

volatile bool a35_ready = true;
static bool uart_ready = false;
static struct lpuart_io lpuart2;

volatile bool trdc_set = false;

static inline void RGPIO_WritePinNSE(RGPIO_Type *base, uint32_t pin)
{
    base->PCNS |= 1UL << pin;
}

static inline uint32_t RGPIO_ReadPinNSE(RGPIO_Type *base, uint32_t pin)
{
    return ((base->PCNS >> pin) & 0x1);
}

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
void app_rpmsg_monitor(struct rpmsg_lite_instance *rpmsgHandle, bool ready, void *rpmsgMonitorParam)
{
    if (ready)
    {
        app_create_task();
    }
    else
    {
        app_destroy_task();
    }
}

static TaskHandle_t app_task_handle = NULL;
static TaskHandle_t lpuart2_task_handle = NULL;

static struct rpmsg_lite_instance *volatile my_rpmsg = NULL;

static struct rpmsg_lite_endpoint *volatile my_ept = NULL;
static volatile rpmsg_queue_handle my_queue        = NULL;
void app_destroy_task(void)
{
    if (app_task_handle)
    {
        vTaskDelete(app_task_handle);
        app_task_handle = NULL;
    }

    if (my_ept)
    {
        rpmsg_lite_destroy_ept(my_rpmsg, my_ept);
        my_ept = NULL;
    }

    if (my_queue)
    {
        rpmsg_queue_destroy(my_rpmsg, my_queue);
        my_queue = NULL;
    }

    if (my_rpmsg)
    {
        rpmsg_lite_deinit(my_rpmsg);
        my_rpmsg = NULL;
    }
}

#define RPMSG_TIMEOUT 10

void app_task(void *param)
{
    volatile uint32_t remote_addr;
    void *rx_buf;
    uint32_t len;
    int32_t result;
    void *tx_buf;

    /* Print the initial banner */
    PRINTF("\r\nCM33-elux\r\n");

    my_rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);

    rpmsg_lite_wait_for_link_up(my_rpmsg, RL_BLOCK);

    my_queue = rpmsg_queue_create(my_rpmsg);
    my_ept   = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, my_queue);
    (void)rpmsg_ns_announce(my_rpmsg, my_ept, RPMSG_LITE_NS_ANNOUNCE_STRING, RL_NS_CREATE);

    PRINTF("\r\nNameservice sent, ready for incoming messages...\r\n");

    typedef struct {
        uint32_t size;
        uint32_t ndx;
        uint32_t max_size;
        uint8_t *buf;
    } RxContext;

    RxContext m2u = {.size = 0, .ndx = 0, .max_size = sizeof(buf_m2u), .buf = buf_m2u };
    RxContext u2m = {.size = 0, .ndx = 0, .max_size = sizeof(buf_u2m), .buf = buf_u2m };

    uintptr_t RPMSG_NONBLOCKING = 0; // 0ms timeout == non-blocking

    for (;;)
    {
        // Mailbox RX

        // If length = 0 means no messages pending to be
        // forwarded, i.e. contention not needed. The opposite case,
        // with > 0, means that we're still processing the previous
        // message, so we skip the read (contention)
        if (a35_ready && m2u.size == 0)
        {
            result = rpmsg_queue_recv_nocopy(my_rpmsg, my_queue, (uint32_t *)&remote_addr, (char **)&rx_buf, &len, RPMSG_NONBLOCKING);

            if (result == RL_SUCCESS)
            {
                assert(len <= m2u.max_size);
                memcpy(m2u.buf, rx_buf, len);

                result = rpmsg_queue_nocopy_free(my_rpmsg, rx_buf);

                if (result == 0)
                {
                    m2u.size = len;
                    m2u.ndx = 0;
                    result = rpmsg_queue_nocopy_free(my_rpmsg, rx_buf);
                    assert(result == 0);
                }
                else
                {
                    assert(false);
                }
            }
            // else: contention (we'll retry later)
        }

        // UART TX (mailbox -> UART)

        if (uart_ready && m2u.size > 0)
        {
            uint8_t *buf = m2u.buf + m2u.ndx;
            uint32_t tx = write_lpuart(&lpuart2, buf, m2u.size);

            m2u.ndx += tx;
            m2u.size -= tx;

            //PRINTF("LPUART TX: %u pending, %u actually sent\r\n", m2u.size, tx);
        }

        // UART RX

        if (uart_ready && u2m.size == 0)
        {
            u2m.size = read_lpuart_interrupt(&lpuart2, u2m.buf, u2m.max_size);
            u2m.ndx = 0;
        }

        // Mailbox TX (UART -> mailbox)

        if (a35_ready && u2m.size > 0)
        {
            uint32_t size;
            tx_buf = rpmsg_lite_alloc_tx_buffer(my_rpmsg, &size, RPMSG_NONBLOCKING);

            if (tx_buf != RL_NULL)
            {
                uint8_t *buf = u2m.buf + u2m.ndx;

                if (size > u2m.size)
                    size = u2m.size;

                memcpy(tx_buf, buf, size);

                if (rpmsg_lite_send_nocopy(my_rpmsg, my_ept, remote_addr, tx_buf, size) != 0)
                    assert(false);

                u2m.size -= size;
                u2m.ndx += size;
            }
            // else: contention (we'll retry later)
        }
    }
}

void lpuart2_task(void *param)
{
    /* Set LPUART2 pin mux */
    IOMUXC_SetPinMux(IOMUXC_PTB2_LPUART2_TX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTB2_LPUART2_TX,
                        IOMUXC_PCR_PE_MASK |
                        IOMUXC_PCR_PS_MASK);
    
    IOMUXC_SetPinMux(IOMUXC_PTB3_LPUART2_RX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTB3_LPUART2_RX,
                        IOMUXC_PCR_PE_MASK |
                        IOMUXC_PCR_PS_MASK);
    
    IOMUXC_SetPinMux(IOMUXC_PTB4_LPUART2_CTS_B, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTB4_LPUART2_CTS_B,
                        IOMUXC_PCR_PE_MASK |
                        IOMUXC_PCR_PS_MASK);
    
    IOMUXC_SetPinMux(IOMUXC_PTB5_LPUART2_RTS_B, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTB5_LPUART2_RTS_B,
                        IOMUXC_PCR_PE_MASK |
                        IOMUXC_PCR_PS_MASK);

    while(!trdc_set) {
        SDK_DelayAtLeastUs(1000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    }

    if (RGPIO_ReadPinNSE(GPIOA, 5) != 1 || \
        RGPIO_ReadPinNSE(GPIOB, 0) != 1 || \
        RGPIO_ReadPinNSE(GPIOB, 1) != 1 || \
        RGPIO_ReadPinNSE(GPIOB, 6) != 1 || \
        RGPIO_ReadPinNSE(GPIOC, 0) != 1 || \
        RGPIO_ReadPinNSE(GPIOC, 1) != 1 || \
        RGPIO_ReadPinNSE(GPIOC, 2) != 1 || \
        RGPIO_ReadPinNSE(GPIOC, 3) != 1 || \
        RGPIO_ReadPinNSE(GPIOC, 4) != 1 || \
        RGPIO_ReadPinNSE(GPIOC, 12) != 1 || \
        RGPIO_ReadPinNSE(GPIOC, 18) != 1 || \
        RGPIO_ReadPinNSE(GPIOC, 19) != 1 || \
        RGPIO_ReadPinNSE(GPIOC, 20) != 1 || \
        RGPIO_ReadPinNSE(GPIOC, 21) != 1 || \
        RGPIO_ReadPinNSE(GPIOC, 22) != 1 || \
        RGPIO_ReadPinNSE(GPIOC, 23) != 1)
    {
        RGPIO_WritePinNSE(GPIOA, 5);
        RGPIO_WritePinNSE(GPIOB, 0);
        RGPIO_WritePinNSE(GPIOB, 1);
        RGPIO_WritePinNSE(GPIOB, 6);
        RGPIO_WritePinNSE(GPIOC, 0);
        RGPIO_WritePinNSE(GPIOC, 1);
        RGPIO_WritePinNSE(GPIOC, 2);
        RGPIO_WritePinNSE(GPIOC, 3);
        RGPIO_WritePinNSE(GPIOC, 4);
        RGPIO_WritePinNSE(GPIOC, 12);
        RGPIO_WritePinNSE(GPIOC, 18);
        RGPIO_WritePinNSE(GPIOC, 19);
        RGPIO_WritePinNSE(GPIOC, 20);
        RGPIO_WritePinNSE(GPIOC, 21);
        RGPIO_WritePinNSE(GPIOC, 22);
        RGPIO_WritePinNSE(GPIOC, 23);
    }

    if(!init_lpuart(&lpuart2, LPUART2, 115200)) {
        PRINTF("LPUART Initialization failed\r\n");
        while(1) {

        }
    }

    uart_ready = true;

    for (;;)
    {
    }
}

void app_create_task(void)
{
    if (app_task_handle == NULL &&
        xTaskCreate(app_task, "APP_TASK", APP_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &app_task_handle) != pdPASS)
    {
        PRINTF("\r\nFailed to create application task\r\n");
        for (;;)
            ;
    }

    if(lpuart2_task_handle == NULL &&
        xTaskCreate(lpuart2_task, "LPUART2_TASK", APP_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &lpuart2_task_handle) != pdPASS)
    {
        PRINTF("\r\nFailed to create lpuart2 task\r\n");
        for (;;)
            ;
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    const cgc_rtd_sys_clk_config_t g_sysClkConfigFroSource = {
        .divCore = 0, /* Core clock divider. */
        .divBus  = 1, /* Bus clock divider. */
        .divSlow = 3, /* Slow clock divider. */
        .src     = kCGC_RtdSysClkSrcFro, /* System clock source. */
        .locked  = 0, /* Register not locked. */
    };

    /* Initialize standard SDK demo application pins */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_BootClockRUN();

    CLOCK_SetFusionSysClkConfig(&g_sysClkConfigFroSource);

    BOARD_InitDebugConsole();

    Fusion_Init();

    UPOWER_PowerOnMemPart(0U, (uint32_t)kUPOWER_MP1_DMA0);

    CLOCK_SetIpSrcDiv(kCLOCK_Tpm0, kCLOCK_Pcc1BusIpSrcCm33Bus, 1U, 0U);
    CLOCK_SetIpSrcDiv(kCLOCK_Lpi2c0, kCLOCK_Pcc1BusIpSrcCm33Bus, 0U, 0U);
    CLOCK_SetIpSrcDiv(kCLOCK_Lpi2c1, kCLOCK_Pcc1BusIpSrcCm33Bus, 0U, 0U);
    /* Use Pll1Pfd2Div clock source 12.288MHz. */
    CLOCK_SetIpSrc(kCLOCK_Sai0, kCLOCK_Cm33SaiClkSrcPll1Pfd2Div);

    CLOCK_EnableClock(kCLOCK_Dma0Ch16);
    CLOCK_EnableClock(kCLOCK_Dma0Ch17);
    CLOCK_EnableClock(kCLOCK_RgpioA);
    CLOCK_EnableClock(kCLOCK_RgpioB);
    CLOCK_EnableClock(kCLOCK_Wuu0);
    CLOCK_EnableClock(kCLOCK_Bbnsm);

    RESET_PeripheralReset(kRESET_Sai0);
    RESET_PeripheralReset(kRESET_Lpi2c0);
    RESET_PeripheralReset(kRESET_Lpi2c1);
    RESET_PeripheralReset(kRESET_Tpm0);

    APP_SRTM_Init();

    /* register callback for restart the app task when A35 reset */
    APP_SRTM_SetRpmsgMonitor(app_rpmsg_monitor, NULL);

    APP_SRTM_StartCommunication();

#ifdef MCMGR_USED
    /* Initialize MCMGR before calling its API */
    (void)MCMGR_Init();
#endif /* MCMGR_USED */

    app_create_task();
    vTaskStartScheduler();

    PRINTF("Failed to start FreeRTOS on core0.\n");
    for (;;)
        ;
}
