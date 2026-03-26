/*
 * =============================================================================
 * TEST 3: Laser Emitter (GPIO)
 * =============================================================================
 * Purpose: Verify the laser emitter turns on and off via GPIO.
 *
 * What it does:
 *   - Toggles the laser on for 1 second, off for 1 second
 *   - Repeats forever
 *   - Prints state to debug console
 *
 * How to verify:
 *   - You should see the laser dot appear and disappear every second
 *   - CAUTION: Do not look directly into the laser beam
 *
 * Wiring:
 *   Laser S (signal) -> PTB1
 *   Laser - (GND)    -> GND
 *   Laser + (VCC)     -> 3.3V (some modules) or not connected
 * =============================================================================
 */

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_debug_console.h"

/* ---- Pin Configuration ---- */
#define LASER_PORT      PORTB
#define LASER_GPIO      GPIOB
#define LASER_PIN       1U

/* ---- Simple blocking delay ---- */
static void delay_ms(uint32_t ms)
{
    volatile uint32_t count = ms * 6000;
    while (count--) { __NOP(); }
}

/* ---- Main ---- */
int main(void)
{
    gpio_pin_config_t outputConfig = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic  = 0U  /* Start OFF */
    };

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    PRINTF("\r\n========================================\r\n");
    PRINTF("  TEST 3: Laser Emitter (GPIO)\r\n");
    PRINTF("========================================\r\n\r\n");

    /* Enable port clock */
    CLOCK_EnableClock(kCLOCK_PortB);

    /* Configure laser pin */
    PORT_SetPinMux(LASER_PORT, LASER_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(LASER_GPIO, LASER_PIN, &outputConfig);

    PRINTF("[LASER] Pin PTB%d configured as GPIO output\r\n", LASER_PIN);
    PRINTF("[LASER] Starting toggle cycle (1s on, 1s off)...\r\n\r\n");
    PRINTF("[LASER] WARNING: Do not look directly into the laser beam!\r\n\r\n");

    uint32_t cycle = 0;
    while (1) {
        cycle++;

        /* ON */
        GPIO_PinWrite(LASER_GPIO, LASER_PIN, 1U);
        PRINTF("[LASER] ON  (cycle %lu)\r\n", cycle);
        delay_ms(1000);

        /* OFF */
        GPIO_PinWrite(LASER_GPIO, LASER_PIN, 0U);
        PRINTF("[LASER] OFF (cycle %lu)\r\n", cycle);
        delay_ms(1000);
    }
}
