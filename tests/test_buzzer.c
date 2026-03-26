/*
 * =============================================================================
 * TEST 2: Passive Buzzer (PWM)
 * =============================================================================
 * Purpose: Verify the passive buzzer generates audible tones at different
 *          frequencies using TPM PWM output.
 *
 * What it does:
 *   - Cycles through 5 different frequencies (the tones used in PetPal)
 *   - Plays each tone for 500ms, then silence for 200ms
 *   - Repeats forever
 *   - Prints the current frequency to the debug console
 *
 * How to verify:
 *   - You should hear distinct tones changing every ~700ms
 *   - Each tone should sound different in pitch
 *   - If you hear nothing, check: wiring, pin mux, PWM config
 *
 * Wiring:
 *   Buzzer + (long leg)  -> PTC3 (TPM0_CH2)
 *   Buzzer - (short leg) -> GND
 *   (Some buzzer modules have 3 pins: S=signal, +=VCC, -=GND.
 *    For passive buzzers, connect S to PTC3, - to GND. VCC may not be needed.)
 * =============================================================================
 */

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_port.h"
#include "fsl_tpm.h"
#include "fsl_debug_console.h"

/* ---- Pin Configuration ---- */
#define BUZZER_TPM_BASE     TPM0
#define BUZZER_TPM_CHANNEL  2U      /* TPM0_CH2 on PTC3 */
#define BUZZER_PORT         PORTC
#define BUZZER_PIN          3U
#define BUZZER_MUX          kPORT_MuxAlt4   /* TPM0_CH2 alt function */

/* ---- Tone definitions (matching PetPal) ---- */
#define TONE_FOOD_READY     1000    /* Hz */
#define TONE_CALL_PET       1500
#define TONE_WATER_LOW      500
#define TONE_INTERACTION    2000
#define TONE_ALERT          800

/* Timer clock after prescaler (adjust to your clock config) */
#define TIMER_CLK_HZ        62500   /* 8MHz OSCERCLK / 128 prescaler */

/* ---- Simple blocking delay ---- */
static void delay_ms(uint32_t ms)
{
    volatile uint32_t count = ms * 6000;
    while (count--) { __NOP(); }
}

/* ---- Set buzzer tone ---- */
static void buzzer_set_tone(uint16_t frequency_hz)
{
    if (frequency_hz == 0) {
        /* Silence: set duty to 0 */
        BUZZER_TPM_BASE->CONTROLS[BUZZER_TPM_CHANNEL].CnV = 0;
        return;
    }

    /*
     * To change the buzzer frequency, we need to change the TPM MOD value.
     * MOD = (timer_clock / desired_frequency) - 1
     * CnV = MOD / 2 for 50% duty cycle (loudest tone)
     *
     * NOTE: Changing MOD affects ALL channels on this TPM.
     *       In the full PetPal project, servo and buzzer share TPM0.
     *       For this standalone test, we own the entire TPM.
     */
    uint16_t modValue = (TIMER_CLK_HZ / frequency_hz) - 1;

    /* Stop timer, update MOD, restart */
    BUZZER_TPM_BASE->SC &= ~TPM_SC_CMOD_MASK;       /* Stop */
    BUZZER_TPM_BASE->CNT = 0;                         /* Reset counter */
    BUZZER_TPM_BASE->MOD = modValue;                   /* New period */
    BUZZER_TPM_BASE->CONTROLS[BUZZER_TPM_CHANNEL].CnV = modValue / 2;  /* 50% duty */
    BUZZER_TPM_BASE->SC |= TPM_SC_CMOD(1);            /* Restart */
}

/* ---- Main ---- */
int main(void)
{
    tpm_config_t tpmConfig;

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    PRINTF("\r\n========================================\r\n");
    PRINTF("  TEST 2: Passive Buzzer (PWM)\r\n");
    PRINTF("========================================\r\n\r\n");

    /* Enable clocks */
    CLOCK_EnableClock(kCLOCK_PortC);

    /* Configure pin mux for TPM0_CH2 */
    PORT_SetPinMux(BUZZER_PORT, BUZZER_PIN, BUZZER_MUX);
    PRINTF("[BUZZER] Pin PTC%d configured as TPM0_CH%d\r\n", BUZZER_PIN, BUZZER_TPM_CHANNEL);

    /* Set TPM clock source */
    CLOCK_SetTpmClock(1U);  /* OSCERCLK */

    /* Initialize TPM */
    TPM_GetDefaultConfig(&tpmConfig);
    tpmConfig.prescale = kTPM_Prescale_Divide_128;
    TPM_Init(BUZZER_TPM_BASE, &tpmConfig);

    /* Configure channel for edge-aligned PWM */
    BUZZER_TPM_BASE->CONTROLS[BUZZER_TPM_CHANNEL].CnSC =
        TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;  /* Edge-aligned PWM, high-true */
    BUZZER_TPM_BASE->CONTROLS[BUZZER_TPM_CHANNEL].CnV = 0;  /* Start silent */

    /* Set initial MOD and start */
    BUZZER_TPM_BASE->MOD = 1249;  /* Default 50Hz (will be overwritten) */
    TPM_StartTimer(BUZZER_TPM_BASE, kTPM_SystemClock);

    PRINTF("[BUZZER] TPM0 initialized. Timer clock: %d Hz\r\n", TIMER_CLK_HZ);
    PRINTF("[BUZZER] Starting tone cycle...\r\n\r\n");

    /* Tone test sequence */
    const struct {
        uint16_t    freq;
        const char *name;
    } tones[] = {
        { TONE_FOOD_READY,  "Food ready (1000 Hz)" },
        { TONE_CALL_PET,    "Call pet (1500 Hz)"    },
        { TONE_WATER_LOW,   "Water low (500 Hz)"    },
        { TONE_INTERACTION, "Interaction (2000 Hz)"  },
        { TONE_ALERT,       "Alert (800 Hz)"         },
    };
    const int numTones = sizeof(tones) / sizeof(tones[0]);

    int cycle = 0;
    while (1) {
        for (int i = 0; i < numTones; i++) {
            PRINTF("[BUZZER] Playing: %s\r\n", tones[i].name);
            buzzer_set_tone(tones[i].freq);
            delay_ms(500);

            PRINTF("[BUZZER] Silence\r\n");
            buzzer_set_tone(0);
            delay_ms(200);
        }
        cycle++;
        PRINTF("\r\n[BUZZER] === Cycle %d complete ===\r\n\r\n", cycle);
    }
}
