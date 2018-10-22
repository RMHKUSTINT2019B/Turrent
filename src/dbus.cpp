/**
 * Edward ZHANG, Terry ZENG, 20180304
 * @file    dbus.c
 * @brief   Dbus driver and decoder with keyboard and mouse support and safe lock
 */

#include "ch.h"
#include "hal.h"
#include "dbus.h"

static uint8_t rxbuf[DBUS_BUFFER_SIZE];

static thread_reference_t uart_dbus_thread_handler = NULL;

#define UART_DBUS                     &UARTD1

/**
 * @brief   Decode the received DBUS sequence and store it in RC_Ctl struct
 */
static void decryptDBUS(void) {
    uartStartSend(UART_DBUS, 13, "Hello World!");
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
    (void) uartp;
    chSysLockFromISR();
    chThdResumeI(&uart_dbus_thread_handler, MSG_OK);
    chSysUnlockFromISR();
}


/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg =
        {
                NULL, NULL, rxend, NULL, NULL,    //Call-back functions
                9600,                    //Baudrate
                USART_CR1_PCE | USART_CR1_M,                //EVEN Parity bit
                USART_CR2_LBDL,
                0
        };


static THD_WORKING_AREA(uart_dbus_thread_wa, 512);

static THD_FUNCTION(uart_dbus_thread, p) {
    (void) p;
    while (!chThdShouldTerminateX()) {
        uartStopReceive(UART_DBUS);
        uartStartReceive(UART_DBUS, DBUS_BUFFER_SIZE, rxbuf);

        chSysLock();
        auto rxmsg = chThdSuspendTimeoutS(&uart_dbus_thread_handler, TIME_INFINITE);
        chSysUnlock();

        if (rxmsg == MSG_OK) {
            chSysLock();
            decryptDBUS();
            chSysUnlock();
        }
    }
}

/**
 * @brief   Initialize the RC receiver
 */
void RC_init(void) {
    uartStart(UART_DBUS, &uart_cfg);
    dmaStreamRelease(*UART_DBUS.dmatx);
    chThdCreateStatic(uart_dbus_thread_wa, sizeof(uart_dbus_thread_wa), NORMALPRIO + 7, uart_dbus_thread, NULL);
}
