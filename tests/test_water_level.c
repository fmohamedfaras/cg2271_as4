/*
 * =============================================================================
 * TEST 5: Water Level Sensor (ADC)
 * =============================================================================
 * Purpose: Verify the water level sensor reads via ADC and calibrate the
 *          threshold values for your specific setup.
 *
 * What it does:
 *   - Reads the water level ADC every 500ms
 *   - Prints raw ADC value (0-4095) and a simple bar graph
 *   - Shows where the current reading falls relative to PetPal thresholds
 *   - Use this to calibrate WATER_LEVEL_LOW_THRESHOLD in petpal.h
 *
 * How to verify:
 *   - With sensor fully submerged: should read HIGH values (2000-4095)
 *   - With sensor in air (dry): should read LOW values (0-200)
 *   - Partially submerged: should be proportional to water depth
 *   - Note the values for "empty bowl", "low", and "full" — these become
 *     your thresholds in the real project
 *
 * Wiring:
 *   Water sensor AO (analog out) -> PTC1 (ADC0_SE15)
 *   Water sensor VCC             -> 3.3V
 *   Water sensor GND             -> GND
 *   (Some modules have a DO pin — we don't use it, only AO)
 * =============================================================================
 */

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_port.h"
#include "fsl_adc16.h"
#include "fsl_debug_console.h"

/* ---- Pin Configuration ---- */
#define WATER_ADC_BASE      ADC0
#define WATER_ADC_CHANNEL   15U     /* ADC0_SE15 on PTC1 */
#define WATER_ADC_GROUP     0U

/* ---- PetPal thresholds (adjust after calibration!) ---- */
#define WATER_LOW_THRESHOLD     200
#define WATER_CRIT_THRESHOLD    100

/* ---- Moving average filter ---- */
#define AVG_SAMPLES     8
static uint16_t samples[AVG_SAMPLES] = {0};
static uint8_t  sampleIdx = 0;
static uint32_t sampleSum = 0;
static uint8_t  sampleCount = 0;

/* ---- Simple blocking delay ---- */
static void delay_ms(uint32_t ms)
{
    volatile uint32_t count = ms * 6000;
    while (count--) { __NOP(); }
}

/* ---- Read ADC ---- */
static uint16_t adc_read(void)
{
    adc16_channel_config_t chConfig;
    chConfig.channelNumber = WATER_ADC_CHANNEL;
    chConfig.enableInterruptOnConversionCompleted = false;
    chConfig.enableDifferentialConversion = false;

    ADC16_SetChannelConfig(WATER_ADC_BASE, WATER_ADC_GROUP, &chConfig);

    /* Wait for conversion */
    while (!ADC16_GetChannelStatusFlags(WATER_ADC_BASE, WATER_ADC_GROUP)) {}

    return (uint16_t)ADC16_GetChannelConversionValue(WATER_ADC_BASE, WATER_ADC_GROUP);
}

/* ---- Add to moving average ---- */
static uint16_t adc_filtered(uint16_t raw)
{
    sampleSum -= samples[sampleIdx];
    samples[sampleIdx] = raw;
    sampleSum += raw;
    sampleIdx = (sampleIdx + 1) % AVG_SAMPLES;
    if (sampleCount < AVG_SAMPLES) sampleCount++;

    return (uint16_t)(sampleSum / sampleCount);
}

/* ---- Print a simple bar graph ---- */
static void print_bar(uint16_t value, uint16_t max)
{
    int barLen = (int)((uint32_t)value * 40 / max);
    if (barLen > 40) barLen = 40;

    PRINTF("[");
    for (int i = 0; i < 40; i++) {
        if (i < barLen) PRINTF("#");
        else PRINTF(" ");
    }
    PRINTF("]");
}

/* ---- Main ---- */
int main(void)
{
    adc16_config_t adcConfig;

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    PRINTF("\r\n========================================\r\n");
    PRINTF("  TEST 5: Water Level Sensor (ADC)\r\n");
    PRINTF("========================================\r\n\r\n");

    /* Enable ADC clock */
    CLOCK_EnableClock(kCLOCK_Adc0);

    /* Initialize ADC */
    ADC16_GetDefaultConfig(&adcConfig);
    adcConfig.resolution = kADC16_ResolutionSE12Bit;
    adcConfig.enableContinuousConversion = false;
    ADC16_Init(WATER_ADC_BASE, &adcConfig);
    ADC16_EnableHardwareTrigger(WATER_ADC_BASE, false);

    /* Calibrate */
    if (ADC16_DoAutoCalibration(WATER_ADC_BASE) != kStatus_Success) {
        PRINTF("[ADC] Calibration FAILED!\r\n");
    } else {
        PRINTF("[ADC] Calibration OK\r\n");
    }

    PRINTF("[ADC] Channel: ADC0_SE%d (PTC1)\r\n", WATER_ADC_CHANNEL);
    PRINTF("[ADC] Resolution: 12-bit (0-4095)\r\n");
    PRINTF("[ADC] Thresholds: LOW=%d, CRITICAL=%d\r\n\r\n",
           WATER_LOW_THRESHOLD, WATER_CRIT_THRESHOLD);
    PRINTF("[ADC] CALIBRATION GUIDE:\r\n");
    PRINTF("  1. Dip sensor fully in water -> note the HIGH value\r\n");
    PRINTF("  2. Hold sensor in air (dry)  -> note the LOW value\r\n");
    PRINTF("  3. Set thresholds between these values\r\n\r\n");

    uint32_t readNum = 0;

    while (1) {
        readNum++;
        uint16_t raw = adc_read();
        uint16_t filtered = adc_filtered(raw);

        /* Determine status */
        const char *status;
        if (filtered < WATER_CRIT_THRESHOLD) {
            status = "CRITICAL";
        } else if (filtered < WATER_LOW_THRESHOLD) {
            status = "LOW     ";
        } else {
            status = "OK      ";
        }

        /* Print reading */
        PRINTF("#%04lu  Raw: %4d  Avg: %4d  ", readNum, raw, filtered);
        print_bar(filtered, 4095);
        PRINTF("  %s\r\n", status);

        delay_ms(500);
    }
}
