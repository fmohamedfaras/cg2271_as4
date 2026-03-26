/*
 * =============================================================================
 * TEST 6: HC-SR04 Ultrasonic Sensor (Interrupt-Driven)
 * =============================================================================
 * Purpose: Verify the HC-SR04 distance sensor using GPIO interrupts for
 *          echo pulse timing. This is the most complex sensor test.
 *
 * What it does:
 *   - Sends a trigger pulse every 200ms
 *   - Uses GPIO interrupt on both edges of the echo pin to time the pulse
 *   - Calculates distance in cm from pulse width
 *   - Prints distance and a visual bar to the debug console
 *   - Shows detection status based on PetPal thresholds
 *
 * How to verify:
 *   - Point sensor at a wall ~20cm away -> should read ~20cm
 *   - Move hand closer/further -> values should change proportionally
 *   - Place hand < 30cm -> should show "PET NEAR"
 *   - Move beyond 60cm -> should show "PET FAR"
 *   - If all readings are 0 or stuck: check wiring, check 5V power to sensor
 *
 * Wiring:
 *   HC-SR04 Trig -> PTA1 (GPIO output)
 *   HC-SR04 Echo -> PTA2 (GPIO input, interrupt)
 *   HC-SR04 VCC  -> 5V  (MUST be 5V, not 3.3V!)
 *   HC-SR04 GND  -> GND
 *
 * IMPORTANT:
 *   The HC-SR04 Echo pin outputs 5V, but the MCXC444 GPIOs are 3.3V tolerant.
 *   Check your board's datasheet. If not 5V tolerant, use a voltage divider:
 *     Echo -> 1K resistor -> PTA2 -> 2K resistor -> GND
 *   This divides 5V to ~3.3V.
 * =============================================================================
 */

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_debug_console.h"

/* ---- Pin Configuration ---- */
#define TRIG_PORT       PORTA
#define TRIG_GPIO       GPIOA
#define TRIG_PIN        1U

#define ECHO_PORT       PORTA
#define ECHO_GPIO       GPIOA
#define ECHO_PIN        2U

/* ---- PetPal thresholds ---- */
#define PET_NEAR_CM     30
#define PET_FAR_CM      60

/* ---- Echo timing state (set by ISR) ---- */
static volatile uint32_t echoStart    = 0;
static volatile uint32_t echoEnd      = 0;
static volatile uint16_t lastDistance  = 0;
static volatile uint8_t  echoReady    = 0;
static volatile uint32_t isrCount     = 0;

/*
 * Microsecond timer using SysTick.
 * SysTick runs at SystemCoreClock (48MHz).
 * We combine a millisecond counter with SysTick->VAL for sub-ms precision.
 */
static volatile uint32_t msCounter = 0;

/* We override SysTick_Handler to count milliseconds */
void SysTick_Handler(void)
{
    msCounter++;
}

static uint32_t get_micros(void)
{
    uint32_t ms = msCounter;
    uint32_t val = SysTick->VAL;
    uint32_t load = SysTick->LOAD + 1;
    uint32_t cycles_per_us = SystemCoreClock / 1000000;

    /* If ms rolled over while reading, re-read */
    if (msCounter != ms) {
        ms = msCounter;
        val = SysTick->VAL;
    }

    uint32_t us_in_tick = (load - val) / cycles_per_us;
    return (ms * 1000) + us_in_tick;
}

/* ---- Blocking microsecond delay ---- */
static void delay_us(uint32_t us)
{
    volatile uint32_t count = us * 12;  /* ~48MHz */
    while (count--) { __NOP(); }
}

static void delay_ms(uint32_t ms)
{
    volatile uint32_t count = ms * 6000;
    while (count--) { __NOP(); }
}

/* ---- PORTA ISR: Echo pin (both edges) ---- */
void PORTA_IRQHandler(void)
{
    uint32_t flags = GPIO_PortGetInterruptFlags(ECHO_GPIO);

    if (flags & (1U << ECHO_PIN)) {
        isrCount++;

        if (GPIO_PinRead(ECHO_GPIO, ECHO_PIN)) {
            /* Rising edge — echo pulse start */
            echoStart = get_micros();
        } else {
            /* Falling edge — echo pulse end */
            echoEnd = get_micros();
            uint32_t pulseWidth = echoEnd - echoStart;

            /* Sanity check: ignore very short (<100us) or very long (>30ms) pulses */
            if (pulseWidth > 100 && pulseWidth < 30000) {
                lastDistance = (uint16_t)(pulseWidth / 58);
            }
            echoReady = 1;
        }

        GPIO_PortClearInterruptFlags(ECHO_GPIO, 1U << ECHO_PIN);
    }
}

/* ---- Send trigger pulse ---- */
static void ultrasonic_trigger(void)
{
    GPIO_PinWrite(TRIG_GPIO, TRIG_PIN, 1U);
    delay_us(10);
    GPIO_PinWrite(TRIG_GPIO, TRIG_PIN, 0U);
}

/* ---- Print bar graph ---- */
static void print_distance_bar(uint16_t cm)
{
    int barLen = (int)cm;
    if (barLen > 50) barLen = 50;

    PRINTF("[");
    for (int i = 0; i < 50; i++) {
        if (i == PET_NEAR_CM / 2) PRINTF("|");  /* Near threshold marker */
        else if (i == PET_FAR_CM / 2) PRINTF("|");  /* Far threshold marker */
        else if (i < barLen) PRINTF("=");
        else PRINTF(" ");
    }
    PRINTF("]");
}

/* ---- Main ---- */
int main(void)
{
    gpio_pin_config_t outputCfg = { .pinDirection = kGPIO_DigitalOutput, .outputLogic = 0U };
    gpio_pin_config_t inputCfg  = { .pinDirection = kGPIO_DigitalInput,  .outputLogic = 0U };

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    /* Setup SysTick for 1ms interrupts (microsecond timer base) */
    SysTick_Config(SystemCoreClock / 1000);

    PRINTF("\r\n========================================\r\n");
    PRINTF("  TEST 6: HC-SR04 Ultrasonic Sensor\r\n");
    PRINTF("========================================\r\n\r\n");

    /* Enable port clock */
    CLOCK_EnableClock(kCLOCK_PortA);

    /* Configure Trig pin (output) */
    PORT_SetPinMux(TRIG_PORT, TRIG_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(TRIG_GPIO, TRIG_PIN, &outputCfg);

    /* Configure Echo pin (input, interrupt on both edges) */
    PORT_SetPinMux(ECHO_PORT, ECHO_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(ECHO_GPIO, ECHO_PIN, &inputCfg);
    PORT_SetPinInterruptConfig(ECHO_PORT, ECHO_PIN, kPORT_InterruptEitherEdge);

    /* Enable PORTA interrupt */
    EnableIRQ(PORTA_IRQn);

    PRINTF("[ULTRASONIC] Trig: PTA%d, Echo: PTA%d\r\n", TRIG_PIN, ECHO_PIN);
    PRINTF("[ULTRASONIC] Thresholds: NEAR < %d cm, FAR > %d cm\r\n", PET_NEAR_CM, PET_FAR_CM);
    PRINTF("[ULTRASONIC] Triggering every 200ms...\r\n\r\n");
    PRINTF("[ULTRASONIC] If stuck at 0cm, check:\r\n");
    PRINTF("  - HC-SR04 VCC is connected to 5V (not 3.3V)\r\n");
    PRINTF("  - Echo pin wiring is correct\r\n");
    PRINTF("  - Echo pin is interrupt-capable (check PORTA IRQ)\r\n\r\n");

    uint32_t triggerCount = 0;

    while (1) {
        /* Send trigger */
        triggerCount++;
        ultrasonic_trigger();

        /* Wait for echo (with timeout) */
        delay_ms(200);

        if (echoReady) {
            echoReady = 0;
            uint16_t dist = lastDistance;

            /* Determine status */
            const char *status;
            if (dist < PET_NEAR_CM) {
                status = "** PET NEAR **";
            } else if (dist > PET_FAR_CM) {
                status = "   PET FAR   ";
            } else {
                status = "  TRANSITION  ";
            }

            PRINTF("#%04lu  Dist: %3d cm  ", triggerCount, dist);
            print_distance_bar(dist);
            PRINTF("  %s  (ISR: %lu)\r\n", status, isrCount);
        } else {
            PRINTF("#%04lu  Dist: --- cm  [  NO ECHO RECEIVED  ]  (ISR: %lu)\r\n",
                   triggerCount, isrCount);
        }
    }
}
