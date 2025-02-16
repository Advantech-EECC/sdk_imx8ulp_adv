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
#include "version.h"

//#define DEBUG_MBOX
//#define DEBUG_UART
//#define DEBUG_RXTX_TASK_MAIN_LOOP
#define DEBUG_A35_ONLINE_TRANSITION

#ifdef DEBUG_UART
#define PRINTF_UART PRINTF
#else
#define PRINTF_UART(...) do {} while (false)
#endif

#ifdef DEBUG_MBOX
#define PRINTF_MBOX PRINTF
#else
#define PRINTF_MBOX(...) do {} while (false)
#endif

extern void app_create_task(void);
extern void app_destroy_task(void);
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RPMSG_LITE_LINK_ID            (RL_PLATFORM_IMX8ULP_M33_A35_USER_LINK_ID)
#define RPMSG_LITE_SHMEM_BASE         (VDEV1_VRING_BASE)
#define RPMSG_LITE_NS_ANNOUNCE_STRING "rpmsg-virtual-tty-channel"
#define RPMSG_LITE_MASTER_IS_LINUX


// Valid rates e.g.
// 9600 19200 38400 57600 115200 230400 460800 921600 1000000 1500000 2000000
#define APP_LPUART2_BAUDRATE 115200

#define APP_TASK_STACK_SIZE (256)
#ifndef LOCAL_EPT_ADDR
#define LOCAL_EPT_ADDR (30)
#endif

typedef struct {
    uint32_t size;
    uint32_t ndx;
    uint32_t max_size;
    uint8_t *buf;
} RxTxContext;

/* Globals */

// Mailbox max payload set to 496
static uint8_t buf_m2u[RL_BUFFER_PAYLOAD_SIZE(0)]; // mailbox -> uart buffer
static uint8_t buf_u2m[RL_BUFFER_PAYLOAD_SIZE(0)]; // uart -> mailbox buffer

volatile bool a35_ready = true;
volatile bool a35_wakeup_signal = false;
volatile bool uart_ready = false;
volatile bool task_stop_rq = false;
volatile bool task_running = false;
volatile bool trdc_set = false;

static struct lpuart_io lpuart2;

static const uintptr_t RPMSG_NONBLOCKING = 0; // 0ms timeout == non-blocking

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
void app_rpmsg_monitor(struct rpmsg_lite_instance *rpmsgHandle, bool ready,
                       void *rpmsgMonitorParam)
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

void app_ap_online_state_change(bool ready, void *param)
{
    if (ready)
    {
        // Instead of changing the 'a35_ready' immediatly, we signal it,
        // so it can be handled in the 'app_task' loop
        a35_wakeup_signal = true;

#ifdef DEBUG_A35_ONLINE_TRANSITION
        PRINTF("\r\nA35 wake-up notification\r\n");
#endif
    }
    else
    {
#ifdef DEBUG_A35_ONLINE_TRANSITION
        PRINTF("\r\nA35 go to offline notification\r\n");
#endif

        // A35 offline: change the state immediately
        a35_ready = false;
    }

    (void)param;
}

static TaskHandle_t app_task_handle = NULL;
static TaskHandle_t lpuart2_task_handle = NULL;
static struct rpmsg_lite_instance *volatile my_rpmsg = NULL;
static struct rpmsg_lite_endpoint *volatile my_ept = NULL;
static volatile rpmsg_queue_handle my_queue = NULL;
static volatile uint32_t my_rpmsg_remote_addr = 0;

void app_destroy_task(void)
{
    task_stop_rq = true;

    PRINTF("\r\nShutting down tasks\r\n");

    vTaskDelay(pdMS_TO_TICKS(10));

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

    task_stop_rq = false;
    task_running = false;

    PRINTF("\r\nTasks stopped\r\n");
}

void app_subtask_mailbox_rx(RxTxContext *c)
{
    // Mailbox RX

    // If length = 0 means no messages pending to be
    // forwarded, i.e. contention not needed. The opposite case,
    // with > 0, means that we're still processing the previous
    // message, so we skip the read (contention)

    if (c->size > 0)
        return;

    int32_t result = rpmsg_queue_recv(my_rpmsg, my_queue,
                                      (uint32_t *)&my_rpmsg_remote_addr,
                                      (char *)c->buf, c->max_size, &c->size,
                                      RPMSG_NONBLOCKING);

    if (result == RL_SUCCESS)
    {
        c->ndx = 0;

        static bool hello_pending = true;

        PRINTF_MBOX("Mailbox RX: %d bytes\r\n", c->size);

        if (hello_pending) {
            const uint32_t first_msg_sz = 12;
            const char *first_msg = "hello world!";

            if (c->size == first_msg_sz &&
                memcmp(first_msg, c->buf, first_msg_sz) == 0)
            {
                // if the first message is the 'hello', we
                // remove it (we'll change this once gets
                // removed from the Linux kernel driver side)
                c->size = 0;
            }

            hello_pending = false;
        }
    }
    else
    {
        // else: contention (we'll retry later)
        if (result != RL_ERR_NO_BUFF)
            PRINTF_MBOX("Mailbox RX: %08x\r\n", (unsigned)result);
    }
}

void app_subtask_mailbox_tx(RxTxContext *c)
{
    // Mailbox TX (UART -> mailbox)

    if (c->size == 0 || my_rpmsg_remote_addr == 0)
        return;

    char *buf = (char *)c->buf;
    int32_t result = rpmsg_lite_send(my_rpmsg, my_ept, my_rpmsg_remote_addr,
                                     buf, c->size, RPMSG_NONBLOCKING);

    if (result == RL_SUCCESS)
    {
        PRINTF_MBOX("Mailbox TX %u\r\n", c->size);

        c->size = 0;
        c->ndx = 0;
    }
    // else: contention (we'll retry later)
}

void app_subtask_uart_rx(RxTxContext *c)
{
    // UART RX

    if (c->size > 0)
        return;

    c->size = read_lpuart_interrupt(&lpuart2, c->buf + c->ndx, c->max_size);
    c->ndx = 0;

    if (c->size > 0)
        PRINTF_UART("LPUART RX %d\r\n", c->size);
}

void app_subtask_uart_tx(RxTxContext *c)
{
    // UART TX (mailbox -> UART)

    if (c->size > 0)
    {
        uint32_t tx;

        do {
            uint8_t *buf = c->buf + c->ndx;
            tx = write_lpuart(&lpuart2, buf, c->size - c->ndx);
            c->ndx += tx;
        } while (tx > 0 && c->ndx != c->size);

        if (c->ndx == c->size)
        {
            // prepare ndx for next transfer
            c->ndx = 0;
            c->size = 0;
        }

        PRINTF_UART("LPUART TX %u left %u\r\n", tx, c->size);
    }
}

void app_task(void *param)
{
    /* Print the initial banner */
    PRINTF("\r\nCM33-iMX8ULP-adv " CM33_IMX8ULP_ADV_VER "\r\n");

    my_rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE,
                                      RPMSG_LITE_LINK_ID, RL_NO_FLAGS);

    rpmsg_lite_wait_for_link_up(my_rpmsg, RL_BLOCK);

    my_queue = rpmsg_queue_create(my_rpmsg);
    my_ept = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR,
                                   rpmsg_queue_rx_cb, my_queue);

    (void)rpmsg_ns_announce(my_rpmsg, my_ept, RPMSG_LITE_NS_ANNOUNCE_STRING,
                            RL_NS_CREATE);

    PRINTF("\r\nNameservice sent, ready for incoming messages...\r\n");

    RxTxContext m2u = {.size = 0, .ndx = 0, .max_size = sizeof(buf_m2u),
                       .buf = buf_m2u };
    RxTxContext u2m = {.size = 0, .ndx = 0, .max_size = sizeof(buf_u2m),
                       .buf = buf_u2m };

    for (uint32_t cnt = 0;; cnt++)
    {
        if (a35_wakeup_signal)
        {
            if (!a35_ready)
            {
                const uint32_t delay_ms = 1000;

                // A35 wake up signal: wait for 1s before making it effective
                vTaskDelay(pdMS_TO_TICKS(delay_ms));
                a35_ready = true;

#ifdef DEBUG_A35_ONLINE_TRANSITION
               PRINTF("\r\nA35 is back online (delayed %u ms)\r\n", delay_ms);
#endif
            }
            a35_wakeup_signal = false;
        }

        if (!a35_ready || !uart_ready || task_stop_rq)
        {
#ifdef DEBUG_RXTX_TASK_MAIN_LOOP
            PRINTF("A35: %d LPUART2: %d STOP_RQ: %d [%u]\r\n",
                   a35_ready, uart_ready, task_stop_rq, cnt);
            vTaskDelay(pdMS_TO_TICKS(1000));
#else
            vTaskDelay(pdMS_TO_TICKS(1));
#endif
            continue;
        }

#ifdef DEBUG_RXTX_TASK_MAIN_LOOP
        PRINTF("A35: %d LPUART2: %d [%u]\r\n", a35_ready, uart_ready, cnt);
        vTaskDelay(pdMS_TO_TICKS(1000));
#endif

        app_subtask_mailbox_rx(&m2u);
        app_subtask_uart_tx(&m2u);
        app_subtask_uart_rx(&u2m);

        if (!a35_ready)
            continue;

        app_subtask_mailbox_tx(&u2m);
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

    if(!init_lpuart(&lpuart2, LPUART2, APP_LPUART2_BAUDRATE)) {
        PRINTF("LPUART Initialization failed\r\n");
        while(1) {

        }
    }

    uart_ready = true;

    for (;;)
    {
        // The work is done in the ISR: this task must leave all the
        // time to the 'app_task', as its busy loop has real-time
        // requirements below the task switching smallest tick

        taskYIELD();
    }
}

void app_create_task(void)
{
    if (task_running)
    {
        PRINTF("\r\nTask restart (TSNH)\r\n");

        app_destroy_task();
    }

    task_running = true;

    PRINTF("\r\nCreate tasks\r\n");

    if (app_task_handle == NULL &&
        xTaskCreate(app_task, "APP_TASK", APP_TASK_STACK_SIZE, NULL,
                    tskIDLE_PRIORITY + 1, &app_task_handle) != pdPASS)
    {
        PRINTF("\r\nFailed to create application task\r\n");
        for (;;)
            ;
    }

    if(lpuart2_task_handle == NULL &&
        xTaskCreate(lpuart2_task, "LPUART2_TASK", APP_TASK_STACK_SIZE, NULL,
                    tskIDLE_PRIORITY + 1, &lpuart2_task_handle) != pdPASS)
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

    rgpio_pin_config_t pin_config = {
        kRGPIO_DigitalInput,
        0,
    };

    /* Initialize standard SDK demo application pins */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();

    IOMUXC_SetPinMux(IOMUXC_PTB0_PTB0, 0U);

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

    RGPIO_SetPinInterruptConfig(GPIOB, 0U, kRGPIO_InterruptOutput2, kRGPIO_InterruptFallingEdge);
    RGPIO_PinInit(GPIOB, 0U, &pin_config);

    /* register callback for restart the app task when A35 reset */
    APP_SRTM_SetRpmsgMonitor(app_rpmsg_monitor, NULL);
    APP_SRTM_SetApOnlineChangeState(app_ap_online_state_change, NULL);

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
