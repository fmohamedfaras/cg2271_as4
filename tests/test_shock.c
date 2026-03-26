/*
 * =============================================================================
 * TEST 7: Shock / Vibration Sensor (Interrupt-Driven)
 * =============================================================================
 * Purpose: Verify the shock sensor triggers interrupts when tapped, with
 *          software debouncing to prevent multiple triggers from one tap.
 *
 * What it does:
 *   - Configures GPIO interrupt on falling edge of shock sensor digital output
 *   - Counts and timestamps each valid (debounced) tap event
 *   - Shows rejected (bounced) events so you can tune the debounce window
 *   - Prints a running event log to the debug console
 *
 * How to verify:
 *   - Tap the sensor gently -> should register ONE event per tap
 *   - Tap rapidly -> events within debounce window should be rejected
 *   - If no events: check wiring, try adjusting the onboard potentiometer
 *     (some shock modules have a sensitivity trim pot)
 *   - If too many events per tap: increase DEBOUNCE_MS
 *
 * Wiring:
 *   Shock sensor DO (digital out) -> PTB0 (GPIO input, interrupt)
 *   Shock sensor VCC              -> 3.3V
 *   Shock sensor GND              -> GND
 *   (The AO pin is not used — we only need the digital threshold output)
 * =============================================================================
 */

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_debug_console.h"

/* ---- Pin Configuration ---- */
#define SHOCK_PORT      PORTB
#define SHOCK_GPIO      GPIOB
#define SHOCK_PIN       0U

/* ---- Debounce configuration ---- */
#define DEBOUNCE_MS     200     /* Ignore triggers within 200ms of last accepted */

/* ---- ISR state ---- */
static volatile uint32_t acceptedCount  = 0;
static volatile uint32_t rejectedCount  = 0;
static volatile uint32_t totalISRCount  = 0;
static volatile uint32_t lastAcceptedMs = 0;
static volatile uint8_t  eventFlag      = 0;

/* Millisecond counter (incremented by SysTick) */
static volatile uint32_t msCounter = 0;

void SysTick_Handler(void)
{
    msCounter++;
}

/* ---- PORTB ISR: Shock sensor (falling edge) ---- */
void PORTB_IRQHandler(void)
{
    uint32_t flags = GPIO_PortGetInterruptFlags(SHOCK_GPIO);

    if (flags & (1U << SHOCK_PIN)) {
        totalISRCount++;
        uint32_t now = msCounter;

        /* Software debounce */
        if ((now - lastAcceptedMs) >= DEBOUNCE_MS) {
            lastAcceptedMs = now;
            acceptedCount++;
            eventFlag = 1;
        } else {
            rejectedCount++;
        }

        GPIO_PortClearInterruptFlags(SHOCK_GPIO, 1U << SHOCK_PIN);
    }
}

/* ---- Simple blocking delay ---- */
static void delay_ms(uint32_t ms)
{
    volatile uint32_t count = ms * 6000;
    while (count--) { __NOP(); }
}

/* ---- Main ---- */
int main(void)
{
    gpio_pin_config_t inputCfg = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic  = 0U
    };

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    /* Setup SysTick for 1ms interrupts */
    SysTick_Config(SystemCoreClock / 1000);

    PRINTF("\r\n========================================\r\n");
    PRINTF("  TEST 7: Shock / Vibration Sensor\r\n");
    PRINTF("========================================\r\n\r\n");

    /* Enable port clock */
    CLOCK_EnableClock(kCLOCK_PortB);

    /* Configure shock pin (input, interrupt on falling edge) */
    PORT_SetPinMux(SHOCK_PORT, SHOCK_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(SHOCK_GPIO, SHOCK_PIN, &inputCfg);
    PORT_SetPinInterruptConfig(SHOCK_PORT, SHOCK_PIN, kPORT_InterruptFallingEdge);

    /* Enable PORTB interrupt */
    EnableIRQ(PORTB_IRQn);

    PRINTF("[SHOCK] Pin PTB%d configured, interrupt on falling edge\r\n", SHOCK_PIN);
    PRINTF("[SHOCK] Debounce window: %d ms\r\n", DEBOUNCE_MS);
    PRINTF("[SHOCK] Waiting for taps...\r\n\r\n");
    PRINTF("[SHOCK] TROUBLESHOOTING:\r\n");
    PRINTF("  - If no events: adjust module potentiometer for sensitivity\r\n");
    PRINTF("  - If too many per tap: increase DEBOUNCE_MS\r\n");
    PRINTF("  - Check DO pin with multimeter: should toggle on tap\r\n\r\n");

    /* Read and display the idle state of the pin */
    uint8_t idleState = GPIO_PinRead(SHOCK_GPIO, SHOCK_PIN);
    PRINTF("[SHOCK] Idle pin state: %d (expecting 1 for active-low sensor)\r\n\r\n", idleState);

    uint32_t lastPrinted = 0;

    while (1) {
        /* Check for new accepted event */
        if (eventFlag) {
            eventFlag = 0;
            PRINTF("[SHOCK] TAP #%lu detected at %lu ms  "
                   "(total ISR: %lu, accepted: %lu, rejected: %lu)\r\n",
                   acceptedCount, lastAcceptedMs,
                   totalISRCount, acceptedCount, rejectedCount);
        }

        /* Periodic status update every 5 seconds */
        if ((msCounter - lastPrinted) >= 5000) {
            lastPrinted = msCounter;
            uint8_t pinState = GPIO_PinRead(SHOCK_GPIO, SHOCK_PIN);
            PRINTF("[SHOCK] Status: pin=%d, total_ISR=%lu, accepted=%lu, rejected=%lu  "
                   "(uptime: %lu s)\r\n",
                   pinState, totalISRCount, acceptedCount, rejectedCount,
                   msCounter / 1000);
        }

        delay_ms(10);
    }
}
