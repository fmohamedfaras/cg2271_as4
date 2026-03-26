/*
 * =============================================================================
 * TEST 1: UART Communication
 * =============================================================================
 * Purpose: Verify bi-directional UART between MCXC444 and ESP32.
 *          This is the FIRST test to run because UART is your primary
 *          debugging tool once you move past the debug console.
 *
 * What it does:
 *   - Sends a counter message to ESP32 every second
 *   - Echoes back any bytes received from ESP32
 *   - Prints all activity to the debug console
 *
 * How to verify:
 *   - Open Arduino Serial Monitor on the ESP32 side
 *   - You should see "MCXC444 HELLO #1", "MCXC444 HELLO #2", etc.
 *   - Type something in the ESP32 Serial Monitor and send it
 *   - The MCXC444 debug console should print what it received
 *
 * ESP32 side: Upload a simple sketch that bridges Serial and Serial2:
 *   void setup() {
 *     Serial.begin(115200);
 *     Serial2.begin(115200, SERIAL_8N1, 16, 17);
 *   }
 *   void loop() {
 *     while (Serial2.available()) Serial.write(Serial2.read());
 *     while (Serial.available()) Serial2.write(Serial.read());
 *   }
 * =============================================================================
 */

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_lpuart.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"

/* ---- Pin Configuration ---- */
#define ESP_UART_BASE       LPUART0
#define ESP_UART_BAUDRATE   115200U
#define ESP_UART_TX_PORT    PORTA
#define ESP_UART_TX_PIN     14U
#define ESP_UART_RX_PORT    PORTA
#define ESP_UART_RX_PIN     15U
#define ESP_UART_CLKSRC     kCLOCK_Osc0ErClk
#define ESP_UART_IRQn       LPUART0_IRQn

/* ---- Receive buffer ---- */
#define RX_BUF_SIZE         64
static volatile uint8_t  rxBuf[RX_BUF_SIZE];
static volatile uint16_t rxHead = 0;
static volatile uint16_t rxTail = 0;

/* ---- UART RX Interrupt Handler ---- */
void LPUART0_IRQHandler(void)
{
    if (LPUART_GetStatusFlags(ESP_UART_BASE) & kLPUART_RxDataRegFullFlag) {
        uint8_t data = LPUART_ReadByte(ESP_UART_BASE);
        uint16_t next = (rxHead + 1) % RX_BUF_SIZE;
        if (next != rxTail) {
            rxBuf[rxHead] = data;
            rxHead = next;
        }
    }
    LPUART_ClearStatusFlags(ESP_UART_BASE,
        kLPUART_RxOverrunFlag | kLPUART_FramingErrorFlag);
}

/* ---- Simple blocking delay ---- */
static void delay_ms(uint32_t ms)
{
    /* Approximate busy-wait for 48MHz core */
    volatile uint32_t count = ms * 6000;
    while (count--) { __NOP(); }
}

/* ---- Send string over UART ---- */
static void uart_send_string(const char *str)
{
    while (*str) {
        LPUART_WriteByte(ESP_UART_BASE, (uint8_t)*str++);
        /* Wait for TX buffer empty */
        while (!(LPUART_GetStatusFlags(ESP_UART_BASE) & kLPUART_TxDataRegEmptyFlag)) {}
    }
}

/* ---- Main ---- */
int main(void)
{
    lpuart_config_t uartConfig;

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    PRINTF("\r\n========================================\r\n");
    PRINTF("  TEST 1: UART Communication\r\n");
    PRINTF("========================================\r\n\r\n");

    /* Enable port clocks */
    CLOCK_EnableClock(kCLOCK_PortA);

    /* Configure UART pins */
    PORT_SetPinMux(ESP_UART_TX_PORT, ESP_UART_TX_PIN, kPORT_MuxAlt3);
    PORT_SetPinMux(ESP_UART_RX_PORT, ESP_UART_RX_PIN, kPORT_MuxAlt3);

    /* Set LPUART clock source */
    CLOCK_SetLpuart0Clock(1U);  /* OSCERCLK */

    /* Initialize UART */
    LPUART_GetDefaultConfig(&uartConfig);
    uartConfig.baudRate_Bps = ESP_UART_BAUDRATE;
    uartConfig.enableTx = true;
    uartConfig.enableRx = true;
    LPUART_Init(ESP_UART_BASE, &uartConfig, CLOCK_GetFreq(ESP_UART_CLKSRC));

    /* Enable RX interrupt */
    LPUART_EnableInterrupts(ESP_UART_BASE, kLPUART_RxDataRegFullInterruptEnable);
    EnableIRQ(ESP_UART_IRQn);

    PRINTF("[UART] Initialized at %d baud\r\n", ESP_UART_BAUDRATE);
    PRINTF("[UART] TX on PTA%d, RX on PTA%d\r\n", ESP_UART_TX_PIN, ESP_UART_RX_PIN);
    PRINTF("[UART] Sending counter every second. Type on ESP32 to test RX.\r\n\r\n");

    uint32_t counter = 0;
    char txBuf[64];

    while (1) {
        /* TX: Send a numbered message every second */
        counter++;
        sprintf(txBuf, "MCXC444 HELLO #%lu\r\n", counter);
        uart_send_string(txBuf);
        PRINTF("[TX] Sent: MCXC444 HELLO #%lu\r\n", counter);

        /* RX: Check for received bytes */
        while (rxHead != rxTail) {
            uint8_t byte = rxBuf[rxTail];
            rxTail = (rxTail + 1) % RX_BUF_SIZE;
            PRINTF("[RX] Received: 0x%02X '%c'\r\n", byte,
                   (byte >= 32 && byte < 127) ? byte : '.');
        }

        delay_ms(1000);
    }
}
