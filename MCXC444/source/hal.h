/*
 * =============================================================================
 * PetPal - Hardware Abstraction Layer
 * =============================================================================
 * File:    hal.h
 * Desc:    Pin definitions and hardware driver API for FRDM-MCXC444
 *
 * Pin assignments (adjust to match your actual wiring):
 *   HC-SR04 Trig  : PTA1  (GPIO output)
 *   HC-SR04 Echo  : PTA2  (GPIO input, interrupt on both edges)
 *   Shock sensor  : PTB0  (GPIO input, interrupt on falling edge)
 *   Water level   : PTC1  (ADC0_SE15)
 *   Servo PWM     : PTC2  (TPM0_CH1)
 *   Buzzer PWM    : PTC3  (TPM0_CH2)
 *   Laser GPIO    : PTB1  (GPIO output)
 *   UART TX       : PTA14 (LPUART0_TX) — to ESP32 RX
 *   UART RX       : PTA15 (LPUART0_RX) — from ESP32 TX
 *
 * NOTE: Adjust these pin assignments to your actual board wiring.
 *       The FRDM-MCXC444 datasheet has the pin mux options.
 * =============================================================================
 */

#ifndef HAL_H
#define HAL_H

#include <stdint.h>
#include <stdbool.h>

/* ========================= PIN DEFINITIONS =============================== */

/*
 * IMPORTANT: These are placeholder pin/port definitions.
 * Replace with actual FRDM-MCXC444 register addresses and pin numbers
 * from the MCUXpresso SDK for your specific wiring.
 *
 * The MCXC444 uses PORT/GPIO modules (PORTA, PORTB, etc.)
 * and TPM modules for PWM.
 */

/* Ultrasonic HC-SR04 */
#define ULTRASONIC_TRIG_PORT    PORTA
#define ULTRASONIC_TRIG_GPIO    GPIOA
#define ULTRASONIC_TRIG_PIN     1U
#define ULTRASONIC_ECHO_PORT    PORTA
#define ULTRASONIC_ECHO_GPIO    GPIOA
#define ULTRASONIC_ECHO_PIN     2U

/* Shock / Vibration sensor */
#define SHOCK_PORT              PORTB
#define SHOCK_GPIO              GPIOB
#define SHOCK_PIN               0U

/* Water level sensor (analog) */
#define WATER_ADC_BASE          ADC0
#define WATER_ADC_CHANNEL       15U     /* ADC0_SE15 on PTC1 */
#define WATER_ADC_GROUP         0U

/* Servo motor (PWM) */
#define SERVO_TPM_BASE          TPM0
#define SERVO_TPM_CHANNEL       1U      /* TPM0_CH1 */

/* Passive buzzer (PWM) */
#define BUZZER_TPM_BASE         TPM0
#define BUZZER_TPM_CHANNEL      2U      /* TPM0_CH2 */

/* Laser emitter (GPIO) */
#define LASER_PORT              PORTB
#define LASER_GPIO              GPIOB
#define LASER_PIN               1U

/* UART to ESP32 */
#define ESP_UART_BASE           LPUART0
#define ESP_UART_BAUDRATE       115200U
#define ESP_UART_TX_PORT        PORTA
#define ESP_UART_TX_PIN         14U
#define ESP_UART_RX_PORT        PORTA
#define ESP_UART_RX_PIN         15U


/* ========================= INITIALIZATION ================================ */

/*
 * Initialize all hardware peripherals:
 *   - System clocks
 *   - GPIO pins (ultrasonic trig, echo, shock, laser)
 *   - ADC for water level sensor
 *   - TPM for servo and buzzer PWM
 *   - LPUART for ESP32 communication
 *   - GPIO interrupts for echo and shock pins
 */
void HAL_Init(void);

/* Individual init functions (called by HAL_Init) */
void HAL_GPIO_Init(void);
void HAL_ADC_Init(void);
void HAL_PWM_Init(void);
void HAL_UART_Init(void);
void HAL_Interrupts_Init(void);


/* ========================= ULTRASONIC SENSOR ============================= */

/*
 * Send a 10us trigger pulse on the ultrasonic trigger pin.
 * The echo ISR will handle the response.
 */
void HAL_Ultrasonic_Trigger(void);

/*
 * Get the last measured distance in centimeters.
 * Updated by the echo pin ISR.
 */
uint16_t HAL_Ultrasonic_GetDistance_cm(void);


/* ========================= WATER LEVEL SENSOR ============================ */

/*
 * Read the water level sensor via ADC.
 * Returns a 12-bit ADC value (0-4095).
 */
uint16_t HAL_WaterLevel_Read(void);


/* ========================= SERVO MOTOR =================================== */

/*
 * Set servo position by pulse width in microseconds.
 * Typical range: 500us (full left) to 2500us (full right).
 * Center is approximately 1500us.
 */
void HAL_Servo_SetPosition(uint16_t pulseWidth_us);


/* ========================= BUZZER ======================================== */

/*
 * Play a tone at the specified frequency (Hz).
 * Set frequency to 0 to stop the buzzer.
 */
void HAL_Buzzer_SetTone(uint16_t frequency_hz);

/*
 * Stop the buzzer.
 */
void HAL_Buzzer_Off(void);


/* ========================= LASER EMITTER ================================= */

void HAL_Laser_On(void);
void HAL_Laser_Off(void);


/* ========================= UART (ESP32 COMMUNICATION) ==================== */

/*
 * Send a buffer of bytes over UART to the ESP32.
 */
void HAL_UART_Send(const uint8_t *data, uint16_t length);

/*
 * Check if UART has received data available.
 */
bool HAL_UART_DataAvailable(void);

/*
 * Read one byte from UART receive buffer.
 * Returns true if a byte was read, false if buffer empty.
 */
bool HAL_UART_ReadByte(uint8_t *byte);


/* ========================= UTILITY ======================================= */

/*
 * Blocking microsecond delay (for ultrasonic trigger pulse).
 * Uses SysTick or busy loop — NOT FreeRTOS vTaskDelay.
 */
void HAL_DelayUs(uint32_t us);

/*
 * Get current system tick count in microseconds.
 * Used by ultrasonic ISR for timing echo pulse width.
 */
uint32_t HAL_GetMicros(void);


#endif /* HAL_H */
