/*
 * =============================================================================
 * PetPal - Autonomous Pet Care & Enrichment Station
 * CG2271 Assignment 4 & Mini Project
 * =============================================================================
 * File:    petpal.h
 * Desc:    Shared definitions, enums, structs, and constants
 * =============================================================================
 */

#ifndef PETPAL_H
#define PETPAL_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/* ========================= SYSTEM CONFIGURATION ========================== */

/* Sensor thresholds */
#define ULTRASONIC_PET_NEAR_CM      30      /* Pet considered "near" if < 30cm */
#define ULTRASONIC_PET_FAR_CM       60      /* Pet considered "gone" if > 60cm */
#define WATER_LEVEL_LOW_THRESHOLD   200     /* ADC value below which water is low (0-4095) */
#define WATER_LEVEL_CRIT_THRESHOLD  100     /* Critical water level */
#define TEMP_HIGH_THRESHOLD         33      /* Celsius - too hot for pet */
#define TEMP_LOW_THRESHOLD          18      /* Celsius - too cold for pet */
#define SHOCK_DEBOUNCE_MS           200     /* Debounce window for shock sensor */

/* Timing */
#define SENSOR_POLL_INTERVAL_MS     2000    /* Water level poll interval */
#define PET_TIMEOUT_MS              10000   /* Pet considered "left" after 10s no detection */
#define FEED_TIMEOUT_MS             30000   /* Max feeding window 30s */
#define PLAY_DURATION_MS            60000   /* Default play session 60s */
#define ULTRASONIC_TRIG_PULSE_US    10      /* Trigger pulse width */

/* Servo positions (PWM duty cycle values) */
#define SERVO_FEED_OPEN             2000    /* Pulse width us - gate open */
#define SERVO_FEED_CLOSED           1000    /* Pulse width us - gate closed */
#define SERVO_PLAY_CENTER           1500    /* Pulse width us - laser center */
#define SERVO_PLAY_LEFT             700     /* Pulse width us - laser left limit */
#define SERVO_PLAY_RIGHT            2300    /* Pulse width us - laser right limit */

/* Buzzer tone frequencies (Hz) */
#define TONE_FOOD_READY             1000
#define TONE_CALL_PET               1500
#define TONE_WATER_LOW              500
#define TONE_INTERACTION_ACK        2000
#define TONE_ALERT                  800
#define TONE_NOT_YET                300     /* "Not feeding time yet" */

/* Queue sizes */
#define ACTUATOR_QUEUE_SIZE         10
#define SENSOR_QUEUE_SIZE           10

/* Task stack sizes (in words) */
#define STACK_SIZE_PET_MONITOR      256
#define STACK_SIZE_ACTUATOR_CTRL    256
#define STACK_SIZE_SENSOR_POLL      192
#define STACK_SIZE_UART_COMM        256
#define STACK_SIZE_SCHEDULER        192

/* Task priorities (higher number = higher priority) */
#define PRIORITY_PET_MONITOR        (configMAX_PRIORITIES - 1)  /* Highest */
#define PRIORITY_ACTUATOR_CTRL      (configMAX_PRIORITIES - 2)  /* High */
#define PRIORITY_SENSOR_POLL        (configMAX_PRIORITIES - 3)  /* Medium */
#define PRIORITY_UART_COMM          (configMAX_PRIORITIES - 3)  /* Medium */
#define PRIORITY_SCHEDULER          (configMAX_PRIORITIES - 4)  /* Low */


/* ========================= ENUMERATIONS ================================== */

/* System FSM states */
typedef enum {
    STATE_IDLE = 0,
    STATE_PET_DETECTED,
    STATE_FEEDING,
    STATE_PLAYING,
    STATE_INTERACTION,
    STATE_ALERT
} SystemState_t;

/* Actuator command types */
typedef enum {
    ACT_BUZZER_TONE = 0,    /* Play a tone at given frequency */
    ACT_BUZZER_OFF,         /* Silence buzzer */
    ACT_SERVO_POSITION,     /* Set servo to specific position */
    ACT_SERVO_SWEEP,        /* Start servo sweep (play mode) */
    ACT_SERVO_STOP,         /* Stop servo sweep */
    ACT_LASER_ON,           /* Turn laser on */
    ACT_LASER_OFF,          /* Turn laser off */
    ACT_FEED_OPEN,          /* Open food gate */
    ACT_FEED_CLOSE,         /* Close food gate */
    ACT_PLAY_START,         /* Start full play mode (laser + sweep) */
    ACT_PLAY_STOP,          /* Stop full play mode */
    ACT_ALL_STOP            /* Emergency stop all actuators */
} ActuatorCmdType_t;

/* Sensor data types for cloud upload */
typedef enum {
    SENSOR_DISTANCE = 0,
    SENSOR_WATER_LEVEL,
    SENSOR_SHOCK_EVENT,
    SENSOR_STATE_CHANGE
} SensorDataType_t;

/* Event codes sent to ESP32 */
typedef enum {
    EVT_PET_ARRIVED = 0x01,
    EVT_PET_LEFT    = 0x02,
    EVT_FEED_START  = 0x03,
    EVT_FEED_DONE   = 0x04,
    EVT_PLAY_START  = 0x05,
    EVT_PLAY_DONE   = 0x06,
    EVT_INTERACTION = 0x07,
    EVT_WATER_LOW   = 0x08,
    EVT_WATER_CRIT  = 0x09,
    EVT_TEMP_ALERT  = 0x0A,
    EVT_BEGGING     = 0x0B
} EventCode_t;


/* ========================= STRUCTURES ==================================== */

/* Actuator command (sent via xQueueActuatorCmd) */
typedef struct {
    ActuatorCmdType_t type;
    uint16_t          value;     /* Frequency for buzzer, position for servo, etc. */
    uint16_t          duration;  /* Duration in ms (0 = indefinite) */
} ActuatorCmd_t;

/* Sensor data packet (sent via xQueueSensorData for cloud upload) */
typedef struct {
    SensorDataType_t type;
    uint16_t         value;
    TickType_t       timestamp;  /* Tick count when measurement taken */
} SensorData_t;

/* Feeding schedule entry */
typedef struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t enabled;
} FeedSchedule_t;

/* System status (shared, protected by mutex where needed) */
typedef struct {
    SystemState_t   currentState;
    uint16_t        lastDistance_cm;
    uint16_t        lastWaterLevel;
    uint8_t         petPresent;
    uint8_t         waterLow;
    uint32_t        feedCount;
    uint32_t        playCount;
    uint32_t        interactionCount;
    uint32_t        visitCount;
    TickType_t      lastPetSeenTick;
    TickType_t      feedingStartTick;
    TickType_t      playingStartTick;
} SystemStatus_t;


/* ========================= GLOBAL HANDLES ================================ */

/* FreeRTOS synchronization handles (defined in main.c) */
extern SemaphoreHandle_t xSemProximity;
extern SemaphoreHandle_t xSemInteraction;
extern QueueHandle_t     xQueueActuatorCmd;
extern QueueHandle_t     xQueueSensorData;
extern SemaphoreHandle_t xMutexUART;
extern SemaphoreHandle_t xMutexStatus;

/* Shared system status */
extern volatile SystemStatus_t systemStatus;

/* Feed schedule (max 4 entries) */
#define MAX_FEED_SCHEDULES  4
extern FeedSchedule_t feedSchedule[MAX_FEED_SCHEDULES];


#endif /* PETPAL_H */
