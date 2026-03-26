/*
 * =============================================================================
 * TEST 4: Servo Motor (PWM)
 * =============================================================================
 * Purpose: Verify the servo motor responds to PWM signals and moves through
 *          its full range. Tests both the food gate positions and laser
 *          sweep range used in PetPal.
 *
 * What it does:
 *   Phase 1 - Position test: Moves to 5 fixed positions (0, 45, 90, 135, 180 deg)
 *   Phase 2 - Feed gate test: Alternates between CLOSED and OPEN positions
 *   Phase 3 - Sweep test: Continuously sweeps left-to-right (simulates play mode)
 *
 * How to verify:
 *   - Phase 1: Servo should step to 5 distinct angles, pausing at each
 *   - Phase 2: Servo should snap between two positions (food gate open/close)
 *   - Phase 3: Servo should sweep smoothly back and forth
 *
 * Wiring:
 *   Servo signal (orange/white) -> PTC2
 *   Servo VCC (red)             -> 5V (use external 5V if board can't supply enough)
 *   Servo GND (brown/black)     -> GND
 *
 * IMPORTANT: Servos draw significant current. If the servo jitters or resets
 *            the board, power it from an external 5V source with common GND.
 * =============================================================================
 */

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_port.h"
#include "fsl_tpm.h"
#include "fsl_debug_console.h"

/* ---- Pin Configuration ---- */
#define SERVO_TPM_BASE      TPM0
#define SERVO_TPM_CHANNEL   1U      /* TPM0_CH1 on PTC2 */
#define SERVO_PORT          PORTC
#define SERVO_PIN           2U
#define SERVO_MUX           kPORT_MuxAlt4

/* ---- Servo pulse width limits (microseconds) ---- */
#define SERVO_MIN_US        500     /* Full left / 0 degrees */
#define SERVO_MAX_US        2500    /* Full right / 180 degrees */
#define SERVO_CENTER_US     1500    /* Center / 90 degrees */

/* PetPal-specific positions */
#define SERVO_FEED_CLOSED   1000
#define SERVO_FEED_OPEN     2000
#define SERVO_PLAY_LEFT     700
#define SERVO_PLAY_RIGHT    2300

/* Timer clock: 8MHz / 128 = 62500 Hz. At 50Hz PWM, MOD = 1249 */
#define TIMER_CLK_HZ        62500
#define SERVO_PWM_FREQ_HZ   50
#define SERVO_MOD_VALUE      ((TIMER_CLK_HZ / SERVO_PWM_FREQ_HZ) - 1)  /* 1249 */

/* ---- Simple blocking delay ---- */
static void delay_ms(uint32_t ms)
{
    volatile uint32_t count = ms * 6000;
    while (count--) { __NOP(); }
}

/*
 * Set servo position by pulse width in microseconds.
 *
 * With 62500 Hz timer clock:
 *   1 timer tick = 1/62500 = 16us
 *   CnV = pulseWidth_us / 16
 *
 * For 50Hz PWM (20ms period):
 *   500us  -> CnV = 31   (0 degrees)
 *   1500us -> CnV = 94   (90 degrees)
 *   2500us -> CnV = 156  (180 degrees)
 */
static void servo_set_position(uint16_t pulseWidth_us)
{
    if (pulseWidth_us < SERVO_MIN_US) pulseWidth_us = SERVO_MIN_US;
    if (pulseWidth_us > SERVO_MAX_US) pulseWidth_us = SERVO_MAX_US;

    uint16_t cnv = pulseWidth_us / 16;
    SERVO_TPM_BASE->CONTROLS[SERVO_TPM_CHANNEL].CnV = cnv;
}

/* Convert pulse width to approximate degrees for display */
static uint16_t pulse_to_degrees(uint16_t pulseWidth_us)
{
    if (pulseWidth_us < SERVO_MIN_US) return 0;
    if (pulseWidth_us > SERVO_MAX_US) return 180;
    return (uint16_t)(((uint32_t)(pulseWidth_us - SERVO_MIN_US) * 180) /
                      (SERVO_MAX_US - SERVO_MIN_US));
}

/* ---- Main ---- */
int main(void)
{
    tpm_config_t tpmConfig;

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    PRINTF("\r\n========================================\r\n");
    PRINTF("  TEST 4: Servo Motor (PWM)\r\n");
    PRINTF("========================================\r\n\r\n");

    /* Enable clocks */
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_SetTpmClock(1U);  /* OSCERCLK */

    /* Configure pin mux */
    PORT_SetPinMux(SERVO_PORT, SERVO_PIN, SERVO_MUX);

    /* Initialize TPM for 50Hz servo PWM */
    TPM_GetDefaultConfig(&tpmConfig);
    tpmConfig.prescale = kTPM_Prescale_Divide_128;
    TPM_Init(SERVO_TPM_BASE, &tpmConfig);

    /* Configure channel for edge-aligned PWM */
    SERVO_TPM_BASE->CONTROLS[SERVO_TPM_CHANNEL].CnSC =
        TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    SERVO_TPM_BASE->CONTROLS[SERVO_TPM_CHANNEL].CnV = SERVO_CENTER_US / 16;

    /* Set MOD for 50Hz */
    SERVO_TPM_BASE->MOD = SERVO_MOD_VALUE;

    /* Start timer */
    TPM_StartTimer(SERVO_TPM_BASE, kTPM_SystemClock);

    PRINTF("[SERVO] TPM0_CH%d on PTC%d, 50Hz PWM\r\n", SERVO_TPM_CHANNEL, SERVO_PIN);
    PRINTF("[SERVO] MOD = %d, Timer clock = %d Hz\r\n\r\n", SERVO_MOD_VALUE, TIMER_CLK_HZ);

    while (1) {
        /* ============ Phase 1: Fixed position test ============ */
        PRINTF("=== PHASE 1: Fixed Position Test ===\r\n");
        const uint16_t positions[] = { 500, 1000, 1500, 2000, 2500 };
        const int numPos = sizeof(positions) / sizeof(positions[0]);

        for (int i = 0; i < numPos; i++) {
            uint16_t deg = pulse_to_degrees(positions[i]);
            PRINTF("[SERVO] Moving to %d us (~%d deg)\r\n", positions[i], deg);
            servo_set_position(positions[i]);
            delay_ms(1500);  /* Wait for servo to reach position */
        }

        PRINTF("\r\n");
        delay_ms(1000);

        /* ============ Phase 2: Feed gate test ============ */
        PRINTF("=== PHASE 2: Feed Gate Test ===\r\n");
        for (int i = 0; i < 4; i++) {
            PRINTF("[SERVO] Gate CLOSED (%d us)\r\n", SERVO_FEED_CLOSED);
            servo_set_position(SERVO_FEED_CLOSED);
            delay_ms(1500);

            PRINTF("[SERVO] Gate OPEN (%d us)\r\n", SERVO_FEED_OPEN);
            servo_set_position(SERVO_FEED_OPEN);
            delay_ms(1500);
        }

        PRINTF("[SERVO] Gate CLOSED (rest position)\r\n");
        servo_set_position(SERVO_FEED_CLOSED);
        delay_ms(1000);

        PRINTF("\r\n");

        /* ============ Phase 3: Sweep test (play mode) ============ */
        PRINTF("=== PHASE 3: Laser Sweep Test ===\r\n");
        PRINTF("[SERVO] Sweeping %d - %d us for 10 seconds...\r\n",
               SERVO_PLAY_LEFT, SERVO_PLAY_RIGHT);

        uint16_t pos = SERVO_PLAY_LEFT;
        int16_t  step = 30;  /* us per iteration */
        uint32_t sweepStart = 0;
        uint32_t iterations = 0;

        /* Sweep for ~10 seconds (approx 10000ms / 30ms per step) */
        while (iterations < 333) {
            servo_set_position(pos);
            pos += step;

            if (pos >= SERVO_PLAY_RIGHT) {
                pos = SERVO_PLAY_RIGHT;
                step = -step;
            } else if (pos <= SERVO_PLAY_LEFT) {
                pos = SERVO_PLAY_LEFT;
                step = -step;
            }

            delay_ms(30);
            iterations++;
        }

        /* Return to center */
        PRINTF("[SERVO] Sweep done. Returning to center.\r\n\r\n");
        servo_set_position(SERVO_CENTER_US);
        delay_ms(2000);

        PRINTF("=== All phases complete. Restarting... ===\r\n\r\n");
    }
}
