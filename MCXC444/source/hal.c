/*
 * =============================================================================
 * PetPal - Hardware Abstraction Layer Implementation
 * =============================================================================
 * File:    hal.c
 * Desc:    Hardware drivers for FRDM-MCXC444 using MCUXpresso SDK
 *
 * NOTE: This code uses the NXP MCUXpresso SDK (fsl_ drivers).
 *       If your project uses different SDK versions or pin mappings,
 *       adjust the register calls accordingly.
 *
 *       When you import this into MCUXpresso IDE, ensure that the
 *       following SDK components are included in your project:
 *         - fsl_gpio
 *         - fsl_port
 *         - fsl_adc16
 *         - fsl_tpm
 *         - fsl_lpuart
 *         - fsl_clock
 * =============================================================================
 */

#include "hal.h"
#include "petpal.h"

/* MCUXpresso SDK includes */
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_adc16.h"
#include "fsl_tpm.h"
#include "fsl_lpuart.h"
#include "fsl_clock.h"
#include "fsl_common.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"

/* FreeRTOS includes for ISR-safe semaphore operations */
#include "FreeRTOS.h"
#include "semphr.h"


/* ========================= INTERNAL STATE ================================ */

/* Ultrasonic echo timing */
static volatile uint32_t echoStartUs  = 0;
static volatile uint32_t echoEndUs    = 0;
static volatile uint16_t lastDistance  = 0;
static volatile bool     echoReceived = false;

/* External semaphore handles (defined in main.c) */
extern SemaphoreHandle_t xSemProximity;
extern SemaphoreHandle_t xSemInteraction;

/* Shock debouncing */
static volatile TickType_t lastShockTick = 0;

/* Simple microsecond counter using a free-running timer */
static volatile uint32_t microsCounter = 0;

/* UART receive ring buffer */
#define UART_RX_BUF_SIZE    64
static volatile uint8_t  uartRxBuf[UART_RX_BUF_SIZE];
static volatile uint16_t uartRxHead = 0;
static volatile uint16_t uartRxTail = 0;


/* ========================= CLOCK AND PIN SETUP =========================== */

void HAL_Init(void)
{
    /* Board-level init (clocks, debug console, etc.) */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();

    /* Enable port clocks */
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortC);

    /* Initialize all peripherals */
    HAL_GPIO_Init();
    HAL_ADC_Init();
    HAL_PWM_Init();
    HAL_UART_Init();
    HAL_Interrupts_Init();
}


/* ========================= GPIO INITIALIZATION =========================== */

void HAL_GPIO_Init(void)
{
    gpio_pin_config_t outputConfig = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic  = 0U
    };
    gpio_pin_config_t inputConfig = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic  = 0U
    };

    /* --- Ultrasonic Trigger (output) --- */
    PORT_SetPinMux(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(ULTRASONIC_TRIG_GPIO, ULTRASONIC_TRIG_PIN, &outputConfig);

    /* --- Ultrasonic Echo (input, interrupt on both edges) --- */
    PORT_SetPinMux(ULTRASONIC_ECHO_PORT, ULTRASONIC_ECHO_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(ULTRASONIC_ECHO_GPIO, ULTRASONIC_ECHO_PIN, &inputConfig);

    /* --- Shock sensor (input, interrupt on falling edge) --- */
    PORT_SetPinMux(SHOCK_PORT, SHOCK_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(SHOCK_GPIO, SHOCK_PIN, &inputConfig);

    /* --- Laser emitter (output, start OFF) --- */
    PORT_SetPinMux(LASER_PORT, LASER_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(LASER_GPIO, LASER_PIN, &outputConfig);
    GPIO_PinWrite(LASER_GPIO, LASER_PIN, 0U);
}


/* ========================= ADC INITIALIZATION ============================ */

void HAL_ADC_Init(void)
{
    adc16_config_t adcConfig;

    /* Enable ADC0 clock */
    CLOCK_EnableClock(kCLOCK_Adc0);

    ADC16_GetDefaultConfig(&adcConfig);
    adcConfig.resolution = kADC16_ResolutionSE12Bit;
    adcConfig.enableContinuousConversion = false;
    ADC16_Init(WATER_ADC_BASE, &adcConfig);
    ADC16_EnableHardwareTrigger(WATER_ADC_BASE, false);

    /* Calibrate ADC */
    ADC16_DoAutoCalibration(WATER_ADC_BASE);
}


/* ========================= PWM INITIALIZATION ============================ */

void HAL_PWM_Init(void)
{
    tpm_config_t tpmConfig;
    tpm_chnl_pwm_signal_param_t servoParam;
    tpm_chnl_pwm_signal_param_t buzzerParam;

    /* Enable TPM0 clock — use OSCERCLK or MCGIRCLK as TPM clock source */
    CLOCK_SetTpmClock(1U);  /* 1 = OSCERCLK */

    TPM_GetDefaultConfig(&tpmConfig);
    tpmConfig.prescale = kTPM_Prescale_Divide_128;
    TPM_Init(SERVO_TPM_BASE, &tpmConfig);

    /*
     * Servo PWM: 50Hz (20ms period)
     * Buzzer PWM: Will be reconfigured dynamically for different tones
     *
     * For servo on TPM0_CH1:
     *   With 8MHz OSCERCLK / 128 prescaler = 62500 Hz timer clock
     *   For 50Hz PWM: MOD = 62500/50 - 1 = 1249
     *   1ms pulse (0 deg) = duty ~5%, 2ms pulse (180 deg) = duty ~10%
     */

    /* Configure servo channel for edge-aligned PWM */
    PORT_SetPinMux(PORTC, 2U, kPORT_MuxAlt4);  /* PTC2 = TPM0_CH1 */
    servoParam.chnlNumber  = (tpm_chnl_t)SERVO_TPM_CHANNEL;
    servoParam.level       = kTPM_HighTrue;
    servoParam.dutyCyclePercent = 7;  /* ~1.5ms = center position */

    /* Configure buzzer channel */
    PORT_SetPinMux(PORTC, 3U, kPORT_MuxAlt4);  /* PTC3 = TPM0_CH2 */
    buzzerParam.chnlNumber  = (tpm_chnl_t)BUZZER_TPM_CHANNEL;
    buzzerParam.level       = kTPM_HighTrue;
    buzzerParam.dutyCyclePercent = 0;  /* Start silent */

    TPM_SetupPwm(SERVO_TPM_BASE, &servoParam, 1U,
                 kTPM_EdgeAlignedPwm, 50U,
                 CLOCK_GetFreq(kCLOCK_Osc0ErClk));

    TPM_StartTimer(SERVO_TPM_BASE, kTPM_SystemClock);
}


/* ========================= UART INITIALIZATION =========================== */

void HAL_UART_Init(void)
{
    lpuart_config_t uartConfig;

    /* Pin mux for LPUART0 */
    PORT_SetPinMux(ESP_UART_TX_PORT, ESP_UART_TX_PIN, kPORT_MuxAlt3);
    PORT_SetPinMux(ESP_UART_RX_PORT, ESP_UART_RX_PIN, kPORT_MuxAlt3);

    /* Enable LPUART0 clock */
    CLOCK_SetLpuart0Clock(1U);  /* 1 = OSCERCLK */

    LPUART_GetDefaultConfig(&uartConfig);
    uartConfig.baudRate_Bps = ESP_UART_BAUDRATE;
    uartConfig.enableTx = true;
    uartConfig.enableRx = true;

    LPUART_Init(ESP_UART_BASE, &uartConfig,
                CLOCK_GetFreq(kCLOCK_Osc0ErClk));

    /* Enable UART RX interrupt */
    LPUART_EnableInterrupts(ESP_UART_BASE, kLPUART_RxDataRegFullInterruptEnable);
    EnableIRQ(LPUART0_IRQn);
}


/* ========================= INTERRUPT SETUP =============================== */

void HAL_Interrupts_Init(void)
{
    /* Ultrasonic echo: interrupt on both edges (rising = start, falling = end) */
    PORT_SetPinInterruptConfig(ULTRASONIC_ECHO_PORT, ULTRASONIC_ECHO_PIN,
                                kPORT_InterruptEitherEdge);

    /* Shock sensor: interrupt on falling edge (active low) */
    PORT_SetPinInterruptConfig(SHOCK_PORT, SHOCK_PIN,
                                kPORT_InterruptFallingEdge);

    /* Enable PORTA IRQ (for ultrasonic echo) */
    EnableIRQ(PORTA_IRQn);

    /* Enable PORTB IRQ (for shock sensor) */
    EnableIRQ(PORTB_IRQn);
}


/* ========================= ISR HANDLERS ================================== */

/*
 * PORTA IRQ Handler - Ultrasonic Echo Pin
 *
 * This ISR fires on both rising and falling edges of the echo pin.
 * - Rising edge: record start time
 * - Falling edge: compute pulse width → distance, give semaphore
 *
 * Distance (cm) = pulse_width_us / 58
 */
void PORTA_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t flags = GPIO_PortGetInterruptFlags(ULTRASONIC_ECHO_GPIO);

    if (flags & (1U << ULTRASONIC_ECHO_PIN)) {
        /* Check if rising or falling edge */
        if (GPIO_PinRead(ULTRASONIC_ECHO_GPIO, ULTRASONIC_ECHO_PIN)) {
            /* Rising edge — echo pulse started */
            echoStartUs = HAL_GetMicros();
        } else {
            /* Falling edge — echo pulse ended */
            echoEndUs = HAL_GetMicros();
            uint32_t pulseWidth = echoEndUs - echoStartUs;

            /* Convert to cm: speed of sound ~ 343 m/s → 1cm = 58us round trip */
            if (pulseWidth > 0 && pulseWidth < 30000) {  /* Max ~5m range */
                lastDistance = (uint16_t)(pulseWidth / 58);
            }

            echoReceived = true;

            /* Give semaphore to wake tskPetMonitor */
            xSemaphoreGiveFromISR(xSemProximity, &xHigherPriorityTaskWoken);
        }

        /* Clear interrupt flag */
        GPIO_PortClearInterruptFlags(ULTRASONIC_ECHO_GPIO,
                                      1U << ULTRASONIC_ECHO_PIN);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*
 * PORTB IRQ Handler - Shock / Vibration Sensor
 *
 * Fires on falling edge. Software debounce: ignore triggers within
 * SHOCK_DEBOUNCE_MS of the last accepted trigger.
 */
void PORTB_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t flags = GPIO_PortGetInterruptFlags(SHOCK_GPIO);

    if (flags & (1U << SHOCK_PIN)) {
        TickType_t now = xTaskGetTickCountFromISR();

        /* Software debounce */
        if ((now - lastShockTick) >= pdMS_TO_TICKS(SHOCK_DEBOUNCE_MS)) {
            lastShockTick = now;

            /* Give semaphore to wake tskPetMonitor */
            xSemaphoreGiveFromISR(xSemInteraction, &xHigherPriorityTaskWoken);
        }

        /* Clear interrupt flag */
        GPIO_PortClearInterruptFlags(SHOCK_GPIO, 1U << SHOCK_PIN);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*
 * LPUART0 IRQ Handler - UART Receive
 *
 * Stores received bytes into a ring buffer for the UART comm task to process.
 */
void LPUART0_IRQHandler(void)
{
    if (LPUART_GetStatusFlags(ESP_UART_BASE) & kLPUART_RxDataRegFullFlag) {
        uint8_t data = LPUART_ReadByte(ESP_UART_BASE);

        /* Store in ring buffer */
        uint16_t nextHead = (uartRxHead + 1) % UART_RX_BUF_SIZE;
        if (nextHead != uartRxTail) {
            uartRxBuf[uartRxHead] = data;
            uartRxHead = nextHead;
        }
        /* If buffer full, drop the byte (overflow) */
    }

    /* Clear any error flags */
    LPUART_ClearStatusFlags(ESP_UART_BASE,
        kLPUART_RxOverrunFlag | kLPUART_FramingErrorFlag);
}


/* ========================= ULTRASONIC FUNCTIONS ========================== */

void HAL_Ultrasonic_Trigger(void)
{
    /* Send a 10us pulse on the trigger pin */
    GPIO_PinWrite(ULTRASONIC_TRIG_GPIO, ULTRASONIC_TRIG_PIN, 1U);
    HAL_DelayUs(ULTRASONIC_TRIG_PULSE_US);
    GPIO_PinWrite(ULTRASONIC_TRIG_GPIO, ULTRASONIC_TRIG_PIN, 0U);
}

uint16_t HAL_Ultrasonic_GetDistance_cm(void)
{
    return lastDistance;
}


/* ========================= WATER LEVEL FUNCTIONS ========================= */

uint16_t HAL_WaterLevel_Read(void)
{
    adc16_channel_config_t chConfig;
    chConfig.channelNumber  = WATER_ADC_CHANNEL;
    chConfig.enableInterruptOnConversionCompleted = false;
    chConfig.enableDifferentialConversion = false;

    ADC16_SetChannelConfig(WATER_ADC_BASE, WATER_ADC_GROUP, &chConfig);

    /* Wait for conversion complete */
    while (!ADC16_GetChannelStatusFlags(WATER_ADC_BASE, WATER_ADC_GROUP)) {
        /* Busy wait — this is called from a task context, not ISR */
    }

    return (uint16_t)ADC16_GetChannelConversionValue(WATER_ADC_BASE,
                                                      WATER_ADC_GROUP);
}


/* ========================= SERVO FUNCTIONS =============================== */

void HAL_Servo_SetPosition(uint16_t pulseWidth_us)
{
    /*
     * Convert pulse width (us) to TPM channel value.
     * With 62500 Hz timer clock and 50Hz PWM:
     *   MOD = 1249 (set during init)
     *   1 timer tick = 16us
     *   Channel value = pulseWidth_us / 16
     *
     * Clamp to safe servo range.
     */
    if (pulseWidth_us < 500)  pulseWidth_us = 500;
    if (pulseWidth_us > 2500) pulseWidth_us = 2500;

    uint16_t chValue = pulseWidth_us / 16;

    SERVO_TPM_BASE->CONTROLS[SERVO_TPM_CHANNEL].CnV = chValue;
}


/* ========================= BUZZER FUNCTIONS ============================== */

void HAL_Buzzer_SetTone(uint16_t frequency_hz)
{
    if (frequency_hz == 0) {
        HAL_Buzzer_Off();
        return;
    }

    /*
     * Reconfigure TPM for the desired frequency on the buzzer channel.
     * For tone generation, we want 50% duty cycle at the target frequency.
     *
     * Timer clock = 62500 Hz
     * MOD for target freq = (62500 / frequency_hz) - 1
     * Channel value = MOD / 2 (50% duty)
     *
     * NOTE: Since servo and buzzer share TPM0, changing MOD affects servo.
     * For a production system, use separate TPM modules.
     * Workaround: Only play tones when servo is stationary,
     * or use a software-toggled GPIO approach for the buzzer.
     */
    uint32_t timerClk = 62500;
    uint16_t modValue = (uint16_t)(timerClk / frequency_hz) - 1;

    /* Set channel value for ~50% duty at this MOD */
    BUZZER_TPM_BASE->CONTROLS[BUZZER_TPM_CHANNEL].CnV = modValue / 2;
}

void HAL_Buzzer_Off(void)
{
    /* Set duty to 0 to silence */
    BUZZER_TPM_BASE->CONTROLS[BUZZER_TPM_CHANNEL].CnV = 0;
}


/* ========================= LASER FUNCTIONS =============================== */

void HAL_Laser_On(void)
{
    GPIO_PinWrite(LASER_GPIO, LASER_PIN, 1U);
}

void HAL_Laser_Off(void)
{
    GPIO_PinWrite(LASER_GPIO, LASER_PIN, 0U);
}


/* ========================= UART FUNCTIONS ================================ */

void HAL_UART_Send(const uint8_t *data, uint16_t length)
{
    LPUART_WriteBlocking(ESP_UART_BASE, data, length);
}

bool HAL_UART_DataAvailable(void)
{
    return (uartRxHead != uartRxTail);
}

bool HAL_UART_ReadByte(uint8_t *byte)
{
    if (uartRxHead == uartRxTail) {
        return false;   /* Buffer empty */
    }

    *byte = uartRxBuf[uartRxTail];
    uartRxTail = (uartRxTail + 1) % UART_RX_BUF_SIZE;
    return true;
}


/* ========================= UTILITY FUNCTIONS ============================= */

void HAL_DelayUs(uint32_t us)
{
    /*
     * Simple busy-wait delay. For a 48MHz core clock:
     * ~48 cycles per microsecond. The loop overhead is ~4 cycles.
     * Adjust the multiplier for your actual clock speed.
     */
    volatile uint32_t count = us * 12;  /* Approximate for 48MHz */
    while (count--) {
        __NOP();
    }
}

uint32_t HAL_GetMicros(void)
{
    /*
     * Use SysTick or a dedicated free-running timer for microsecond resolution.
     * Simple approach: combine FreeRTOS tick count with SysTick current value.
     *
     * FreeRTOS tick = 1ms (configTICK_RATE_HZ = 1000)
     * SysTick counts down from (SystemCoreClock / 1000 - 1) each tick.
     *
     * microseconds = (tick_count * 1000) + ((reload - current) / cycles_per_us)
     */
    uint32_t ticks = xTaskGetTickCount();
    uint32_t sysTickVal = SysTick->VAL;
    uint32_t sysTickLoad = SysTick->LOAD + 1;
    uint32_t cyclesPerUs = SystemCoreClock / 1000000;

    uint32_t usInTick = (sysTickLoad - sysTickVal) / cyclesPerUs;

    return (ticks * 1000) + usInTick;
}
