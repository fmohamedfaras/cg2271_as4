/*
 * =============================================================================
 * TEST 8: Combined Integration Test (FreeRTOS)
 * =============================================================================
 * Purpose: Verify ALL sensors and actuators working together under FreeRTOS.
 *          This is the final test before integrating the full PetPal FSM.
 *
 * What it does:
 *   - Task 1 (tskSensors):  Triggers ultrasonic, reads water level, prints both
 *   - Task 2 (tskActuators): Cycles through actuator actions on command
 *   - Task 3 (tskMonitor):   Waits for ultrasonic/shock ISR semaphores, prints events
 *   - ISR: Ultrasonic echo (semaphore), Shock sensor (semaphore)
 *   - Semaphore, Queue, and Mutex all exercised
 *
 * How to verify:
 *   - All sensor readings should print continuously
 *   - Wave hand near ultrasonic -> "PROXIMITY EVENT" prints from tskMonitor
 *   - Tap shock sensor -> "SHOCK EVENT" prints from tskMonitor
 *   - Actuators cycle every 3 seconds (buzzer tone, servo sweep, laser toggle)
 *   - No crashes, no freezes, no garbled output (would indicate stack overflow
 *     or mutex issues)
 *
 * This test proves:
 *   [x] Multiple FreeRTOS tasks running concurrently
 *   [x] ISR -> semaphore -> task wakeup path
 *   [x] Queue: sensor task -> actuator task commands
 *   [x] Mutex: protecting shared PRINTF output
 *   [x] Pre-emption and time-slicing working
 *   [x] All hardware peripherals operational simultaneously
 * =============================================================================
 */

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_adc16.h"
#include "fsl_tpm.h"
#include "fsl_debug_console.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/* ========================= PIN DEFINITIONS =============================== */

/* Ultrasonic */
#define TRIG_PORT       PORTA
#define TRIG_GPIO       GPIOA
#define TRIG_PIN        1U
#define ECHO_PORT       PORTA
#define ECHO_GPIO       GPIOA
#define ECHO_PIN        2U

/* Shock */
#define SHOCK_PORT      PORTB
#define SHOCK_GPIO      GPIOB
#define SHOCK_PIN       0U

/* Water Level */
#define WATER_ADC_BASE      ADC0
#define WATER_ADC_CHANNEL   15U
#define WATER_ADC_GROUP     0U

/* Servo */
#define SERVO_TPM_BASE      TPM0
#define SERVO_TPM_CHANNEL   1U
#define SERVO_PORT_C        PORTC
#define SERVO_PIN_C         2U

/* Buzzer */
#define BUZZER_TPM_CHANNEL  2U
#define BUZZER_PORT_C       PORTC
#define BUZZER_PIN_C        3U

/* Laser */
#define LASER_PORT      PORTB
#define LASER_GPIO      GPIOB
#define LASER_PIN       1U

/* Timer clock */
#define TIMER_CLK_HZ    62500
#define SERVO_MOD       1249    /* 50Hz */

/* ========================= DEBOUNCE ====================================== */
#define SHOCK_DEBOUNCE_MS   200

/* ========================= FREERTOS HANDLES ============================== */

static SemaphoreHandle_t xSemProximity   = NULL;
static SemaphoreHandle_t xSemShock       = NULL;
static SemaphoreHandle_t xMutexPrint     = NULL;
static QueueHandle_t     xQueueActCmd    = NULL;

/* ========================= SHARED STATE ================================== */

static volatile uint16_t lastDistance = 0;
static volatile uint32_t echoStartUs = 0;
static volatile TickType_t lastShockTick = 0;
static volatile uint32_t proximityEvents = 0;
static volatile uint32_t shockEvents     = 0;

/* Actuator command structure */
typedef enum { CMD_BUZZER, CMD_SERVO_SWEEP, CMD_LASER_TOGGLE, CMD_SILENCE } CmdType_t;
typedef struct { CmdType_t type; uint16_t value; } ActCmd_t;

/* ========================= MICROSECOND TIMER ============================= */

static volatile uint32_t msCount = 0;

/* NOTE: If FreeRTOS is using SysTick, we use xTaskGetTickCount instead */
static uint32_t get_micros_from_isr(void)
{
    uint32_t ticks = xTaskGetTickCountFromISR();
    uint32_t val = SysTick->VAL;
    uint32_t load = SysTick->LOAD + 1;
    uint32_t cps = SystemCoreClock / 1000000;
    return (ticks * 1000) + ((load - val) / cps);
}

/* ========================= BLOCKING DELAY (for trigger pulse) ============ */

static void delay_us(uint32_t us)
{
    volatile uint32_t c = us * 12;
    while (c--) { __NOP(); }
}

/* ========================= ISR HANDLERS ================================== */

/* Ultrasonic Echo (PORTA) */
void PORTA_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;
    uint32_t flags = GPIO_PortGetInterruptFlags(ECHO_GPIO);

    if (flags & (1U << ECHO_PIN)) {
        if (GPIO_PinRead(ECHO_GPIO, ECHO_PIN)) {
            echoStartUs = get_micros_from_isr();
        } else {
            uint32_t end = get_micros_from_isr();
            uint32_t pw = end - echoStartUs;
            if (pw > 100 && pw < 30000) {
                lastDistance = (uint16_t)(pw / 58);
            }
            xSemaphoreGiveFromISR(xSemProximity, &woken);
        }
        GPIO_PortClearInterruptFlags(ECHO_GPIO, 1U << ECHO_PIN);
    }
    portYIELD_FROM_ISR(woken);
}

/* Shock sensor (PORTB) */
void PORTB_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;
    uint32_t flags = GPIO_PortGetInterruptFlags(SHOCK_GPIO);

    if (flags & (1U << SHOCK_PIN)) {
        TickType_t now = xTaskGetTickCountFromISR();
        if ((now - lastShockTick) >= pdMS_TO_TICKS(SHOCK_DEBOUNCE_MS)) {
            lastShockTick = now;
            xSemaphoreGiveFromISR(xSemShock, &woken);
        }
        GPIO_PortClearInterruptFlags(SHOCK_GPIO, 1U << SHOCK_PIN);
    }
    portYIELD_FROM_ISR(woken);
}

/* ========================= HARDWARE HELPERS ============================== */

static uint16_t adc_read(void)
{
    adc16_channel_config_t ch;
    ch.channelNumber = WATER_ADC_CHANNEL;
    ch.enableInterruptOnConversionCompleted = false;
    ch.enableDifferentialConversion = false;
    ADC16_SetChannelConfig(WATER_ADC_BASE, WATER_ADC_GROUP, &ch);
    while (!ADC16_GetChannelStatusFlags(WATER_ADC_BASE, WATER_ADC_GROUP)) {}
    return (uint16_t)ADC16_GetChannelConversionValue(WATER_ADC_BASE, WATER_ADC_GROUP);
}

static void servo_set(uint16_t pulse_us)
{
    if (pulse_us < 500) pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;
    SERVO_TPM_BASE->CONTROLS[SERVO_TPM_CHANNEL].CnV = pulse_us / 16;
}

static void buzzer_tone(uint16_t freq)
{
    if (freq == 0) {
        SERVO_TPM_BASE->CONTROLS[BUZZER_TPM_CHANNEL].CnV = 0;
        return;
    }
    /* NOTE: In this test we only set the channel value.
       For accurate frequency, we'd need to change MOD (which affects servo).
       This gives an approximate tone for testing purposes. */
    uint16_t mod = SERVO_TPM_BASE->MOD;
    SERVO_TPM_BASE->CONTROLS[BUZZER_TPM_CHANNEL].CnV = mod / 4;
}

static void laser_set(uint8_t on)
{
    GPIO_PinWrite(LASER_GPIO, LASER_PIN, on ? 1U : 0U);
}

/* ========================= SAFE PRINTF =================================== */

#define SAFE_PRINTF(...)  do { \
    if (xSemaphoreTake(xMutexPrint, pdMS_TO_TICKS(50)) == pdTRUE) { \
        PRINTF(__VA_ARGS__); \
        xSemaphoreGive(xMutexPrint); \
    } \
} while(0)

/* ========================= TASK: SENSOR POLLING ========================== */
/*
 * Periodically triggers ultrasonic and reads water level.
 * Demonstrates: periodic task with vTaskDelayUntil, ADC polling, mutex use.
 */
static void tskSensors(void *pv)
{
    (void)pv;
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t cycle = 0;

    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(500));
        cycle++;

        /* Trigger ultrasonic */
        GPIO_PinWrite(TRIG_GPIO, TRIG_PIN, 1U);
        delay_us(10);
        GPIO_PinWrite(TRIG_GPIO, TRIG_PIN, 0U);

        /* Read water level */
        uint16_t water = adc_read();

        /* Print sensor readings */
        SAFE_PRINTF("[SENSORS] #%lu  Dist: %3d cm  Water: %4d  "
                    "(proxEvt: %lu, shockEvt: %lu)\r\n",
                    cycle, lastDistance, water, proximityEvents, shockEvents);
    }
}

/* ========================= TASK: EVENT MONITOR =========================== */
/*
 * Waits for ISR semaphores and logs events.
 * Demonstrates: semaphore-driven task wakeup from ISR, highest priority task.
 */
static void tskMonitor(void *pv)
{
    (void)pv;

    for (;;) {
        /* Check proximity semaphore */
        if (xSemaphoreTake(xSemProximity, pdMS_TO_TICKS(50)) == pdTRUE) {
            proximityEvents++;
            uint16_t dist = lastDistance;

            if (dist < 30) {
                SAFE_PRINTF("[MONITOR] *** PROXIMITY EVENT *** Dist: %d cm — PET NEAR!\r\n", dist);

                /* Send actuator command: buzzer chirp */
                ActCmd_t cmd = { .type = CMD_BUZZER, .value = 1 };
                xQueueSend(xQueueActCmd, &cmd, 0);
            }
        }

        /* Check shock semaphore (non-blocking) */
        if (xSemaphoreTake(xSemShock, 0) == pdTRUE) {
            shockEvents++;
            SAFE_PRINTF("[MONITOR] *** SHOCK EVENT #%lu *** Pet interaction detected!\r\n",
                        shockEvents);

            /* Send actuator command: laser toggle */
            ActCmd_t cmd = { .type = CMD_LASER_TOGGLE, .value = 0 };
            xQueueSend(xQueueActCmd, &cmd, 0);
        }
    }
}

/* ========================= TASK: ACTUATOR CONTROL ======================== */
/*
 * Dequeues commands and drives actuators.
 * Also runs a periodic self-test cycle when idle.
 * Demonstrates: queue-driven task, actuator ownership pattern.
 */
static void tskActuators(void *pv)
{
    (void)pv;
    ActCmd_t cmd;
    uint8_t laserState = 0;
    uint32_t idleCycles = 0;

    /* Self-test actuator sequence */
    typedef enum { SELF_BUZZER, SELF_SWEEP, SELF_LASER, SELF_IDLE } SelfTestPhase_t;
    SelfTestPhase_t phase = SELF_BUZZER;
    TickType_t phaseStart = xTaskGetTickCount();

    for (;;) {
        /* Check for commands from other tasks */
        if (xQueueReceive(xQueueActCmd, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            switch (cmd.type) {
            case CMD_BUZZER:
                SAFE_PRINTF("[ACTUATOR] Buzzer chirp\r\n");
                buzzer_tone(1000);
                vTaskDelay(pdMS_TO_TICKS(150));
                buzzer_tone(0);
                break;

            case CMD_LASER_TOGGLE:
                laserState = !laserState;
                laser_set(laserState);
                SAFE_PRINTF("[ACTUATOR] Laser %s\r\n", laserState ? "ON" : "OFF");
                break;

            case CMD_SILENCE:
                buzzer_tone(0);
                laser_set(0);
                laserState = 0;
                servo_set(1500);
                break;

            default:
                break;
            }
            continue;  /* Restart loop to check for more commands */
        }

        /* Self-test cycle when no commands are pending (every 8 seconds) */
        idleCycles++;
        TickType_t elapsed = xTaskGetTickCount() - phaseStart;

        switch (phase) {
        case SELF_BUZZER:
            if (elapsed > pdMS_TO_TICKS(2000)) {
                SAFE_PRINTF("[ACTUATOR] Self-test: Buzzer tone\r\n");
                buzzer_tone(800);
                vTaskDelay(pdMS_TO_TICKS(300));
                buzzer_tone(0);
                phase = SELF_SWEEP;
                phaseStart = xTaskGetTickCount();
            }
            break;

        case SELF_SWEEP:
            if (elapsed < pdMS_TO_TICKS(3000)) {
                /* Mini sweep */
                uint16_t pos = 1000 + (uint16_t)((elapsed * 1000) / pdMS_TO_TICKS(3000));
                servo_set(pos);
            } else {
                servo_set(1500);
                SAFE_PRINTF("[ACTUATOR] Self-test: Servo sweep done\r\n");
                phase = SELF_LASER;
                phaseStart = xTaskGetTickCount();
            }
            break;

        case SELF_LASER:
            if (elapsed > pdMS_TO_TICKS(1000)) {
                laser_set(1);
                SAFE_PRINTF("[ACTUATOR] Self-test: Laser flash\r\n");
                vTaskDelay(pdMS_TO_TICKS(500));
                laser_set(0);
                phase = SELF_IDLE;
                phaseStart = xTaskGetTickCount();
            }
            break;

        case SELF_IDLE:
            if (elapsed > pdMS_TO_TICKS(5000)) {
                phase = SELF_BUZZER;
                phaseStart = xTaskGetTickCount();
                SAFE_PRINTF("[ACTUATOR] --- Self-test cycle restart ---\r\n");
            }
            break;
        }
    }
}

/* ========================= HARDWARE INIT ================================= */

static void hw_init(void)
{
    gpio_pin_config_t outCfg = { .pinDirection = kGPIO_DigitalOutput, .outputLogic = 0 };
    gpio_pin_config_t inCfg  = { .pinDirection = kGPIO_DigitalInput,  .outputLogic = 0 };
    adc16_config_t adcCfg;
    tpm_config_t tpmCfg;

    /* Clocks */
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_Adc0);
    CLOCK_SetTpmClock(1U);

    /* --- GPIO: Ultrasonic --- */
    PORT_SetPinMux(TRIG_PORT, TRIG_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(TRIG_GPIO, TRIG_PIN, &outCfg);
    PORT_SetPinMux(ECHO_PORT, ECHO_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(ECHO_GPIO, ECHO_PIN, &inCfg);
    PORT_SetPinInterruptConfig(ECHO_PORT, ECHO_PIN, kPORT_InterruptEitherEdge);

    /* --- GPIO: Shock --- */
    PORT_SetPinMux(SHOCK_PORT, SHOCK_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(SHOCK_GPIO, SHOCK_PIN, &inCfg);
    PORT_SetPinInterruptConfig(SHOCK_PORT, SHOCK_PIN, kPORT_InterruptFallingEdge);

    /* --- GPIO: Laser --- */
    PORT_SetPinMux(LASER_PORT, LASER_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(LASER_GPIO, LASER_PIN, &outCfg);

    /* --- ADC: Water level --- */
    ADC16_GetDefaultConfig(&adcCfg);
    adcCfg.resolution = kADC16_ResolutionSE12Bit;
    adcCfg.enableContinuousConversion = false;
    ADC16_Init(WATER_ADC_BASE, &adcCfg);
    ADC16_EnableHardwareTrigger(WATER_ADC_BASE, false);
    ADC16_DoAutoCalibration(WATER_ADC_BASE);

    /* --- TPM: Servo + Buzzer --- */
    PORT_SetPinMux(SERVO_PORT_C, SERVO_PIN_C, kPORT_MuxAlt4);
    PORT_SetPinMux(BUZZER_PORT_C, BUZZER_PIN_C, kPORT_MuxAlt4);
    TPM_GetDefaultConfig(&tpmCfg);
    tpmCfg.prescale = kTPM_Prescale_Divide_128;
    TPM_Init(SERVO_TPM_BASE, &tpmCfg);
    SERVO_TPM_BASE->CONTROLS[SERVO_TPM_CHANNEL].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    SERVO_TPM_BASE->CONTROLS[BUZZER_TPM_CHANNEL].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    SERVO_TPM_BASE->CONTROLS[SERVO_TPM_CHANNEL].CnV = 1500 / 16;  /* Center */
    SERVO_TPM_BASE->CONTROLS[BUZZER_TPM_CHANNEL].CnV = 0;          /* Silent */
    SERVO_TPM_BASE->MOD = SERVO_MOD;
    TPM_StartTimer(SERVO_TPM_BASE, kTPM_SystemClock);

    /* --- Enable interrupts --- */
    EnableIRQ(PORTA_IRQn);
    EnableIRQ(PORTB_IRQn);
}

/* ========================= MAIN ========================================== */

int main(void)
{
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    PRINTF("\r\n========================================\r\n");
    PRINTF("  TEST 8: Combined Integration Test\r\n");
    PRINTF("  (FreeRTOS + All Sensors + All Actuators)\r\n");
    PRINTF("========================================\r\n\r\n");

    /* Initialize all hardware */
    hw_init();
    PRINTF("[INIT] All hardware initialized\r\n");

    /* Create FreeRTOS objects */
    xSemProximity = xSemaphoreCreateBinary();
    xSemShock     = xSemaphoreCreateBinary();
    xMutexPrint   = xSemaphoreCreateMutex();
    xQueueActCmd  = xQueueCreate(8, sizeof(ActCmd_t));

    configASSERT(xSemProximity);
    configASSERT(xSemShock);
    configASSERT(xMutexPrint);
    configASSERT(xQueueActCmd);
    PRINTF("[INIT] FreeRTOS objects created\r\n");

    /* Create tasks */
    xTaskCreate(tskMonitor,   "Mon",  256, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(tskActuators, "Act",  256, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(tskSensors,   "Sens", 256, NULL, configMAX_PRIORITIES - 3, NULL);

    PRINTF("[INIT] Tasks created:\r\n");
    PRINTF("  tskMonitor   (priority %d) - ISR event handler\r\n", configMAX_PRIORITIES - 1);
    PRINTF("  tskActuators (priority %d) - Actuator driver\r\n", configMAX_PRIORITIES - 2);
    PRINTF("  tskSensors   (priority %d) - Sensor polling\r\n", configMAX_PRIORITIES - 3);
    PRINTF("\r\n[INIT] Starting FreeRTOS scheduler...\r\n");
    PRINTF("========================================\r\n\r\n");

    /* Start scheduler */
    vTaskStartScheduler();

    /* Should never reach here */
    for (;;) {}
}

/* ========================= FREERTOS HOOKS ================================ */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    PRINTF("\r\n!!! STACK OVERFLOW in task: %s !!!\r\n", pcTaskName);
    for (;;) {}
}

void vApplicationMallocFailedHook(void)
{
    PRINTF("\r\n!!! MALLOC FAILED !!!\r\n");
    for (;;) {}
}
