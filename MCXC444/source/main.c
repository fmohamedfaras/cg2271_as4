/*
 * =============================================================================
 * PetPal - Main Application (MCXC444 / FreeRTOS)
 * =============================================================================
 * File:    main.c
 * Desc:    FreeRTOS-based pet care station controller
 *
 * Tasks:
 *   tskPetMonitor      - Highest priority. FSM brain. Takes semaphores from
 *                         ISRs, manages state transitions, enqueues actuator cmds.
 *   tskActuatorControl - High priority. Dequeues and executes actuator commands
 *                         (servo, buzzer, laser).
 *   tskSensorPoll      - Medium priority. Periodic ADC read of water level sensor.
 *                         Moving average filter. Pushes data to sensor queue.
 *   tskUARTComm        - Medium priority. Bi-directional packetized UART with ESP32.
 *                         Sends sensor data/events out, receives commands in.
 *   tskScheduler       - Low priority. Manages feeding and play schedules.
 *
 * ISRs:
 *   PORTA_IRQHandler   - Ultrasonic echo pin (both edges) → xSemProximity
 *   PORTB_IRQHandler   - Shock sensor (falling edge, debounced) → xSemInteraction
 *   LPUART0_IRQHandler - UART RX → ring buffer
 *
 * Synchronization:
 *   xSemProximity      - Binary semaphore (ISR_Echo → tskPetMonitor)
 *   xSemInteraction    - Binary semaphore (ISR_Shock → tskPetMonitor)
 *   xQueueActuatorCmd  - Queue of ActuatorCmd_t (multiple producers → tskActuatorControl)
 *   xQueueSensorData   - Queue of SensorData_t (tskSensorPoll → tskUARTComm)
 *   xMutexUART         - Mutex protecting UART TX
 *   xMutexStatus       - Mutex protecting systemStatus shared struct
 * =============================================================================
 */

#include <stdio.h>
#include <string.h>

#include "petpal.h"
#include "hal.h"
#include "protocol.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"


/* ========================= GLOBAL HANDLES ================================ */

SemaphoreHandle_t xSemProximity    = NULL;
SemaphoreHandle_t xSemInteraction  = NULL;
QueueHandle_t     xQueueActuatorCmd = NULL;
QueueHandle_t     xQueueSensorData  = NULL;
SemaphoreHandle_t xMutexUART       = NULL;
SemaphoreHandle_t xMutexStatus     = NULL;

/* Shared system status */
volatile SystemStatus_t systemStatus = {
    .currentState      = STATE_IDLE,
    .lastDistance_cm    = 999,
    .lastWaterLevel    = 4095,
    .petPresent        = 0,
    .waterLow          = 0,
    .feedCount         = 0,
    .playCount         = 0,
    .interactionCount  = 0,
    .visitCount        = 0,
    .lastPetSeenTick   = 0,
    .feedingStartTick  = 0,
    .playingStartTick  = 0
};

/* Feed schedule (default: 8AM and 6PM) */
FeedSchedule_t feedSchedule[MAX_FEED_SCHEDULES] = {
    { .hour = 8,  .minute = 0, .enabled = 1 },
    { .hour = 18, .minute = 0, .enabled = 1 },
    { .hour = 0,  .minute = 0, .enabled = 0 },
    { .hour = 0,  .minute = 0, .enabled = 0 }
};

/* Task handles */
static TaskHandle_t hTaskPetMonitor    = NULL;
static TaskHandle_t hTaskActuatorCtrl  = NULL;
static TaskHandle_t hTaskSensorPoll    = NULL;
static TaskHandle_t hTaskUARTComm      = NULL;
static TaskHandle_t hTaskScheduler     = NULL;

/* Software timer for periodic ultrasonic triggers */
static TimerHandle_t hTimerUltrasonic  = NULL;

/* UART protocol parser */
static Parser_t uartParser;


/* ========================= HELPER FUNCTIONS ============================== */

/*
 * Thread-safe update of system status field.
 */
static void status_update_state(SystemState_t newState)
{
    if (xSemaphoreTake(xMutexStatus, pdMS_TO_TICKS(10)) == pdTRUE) {
        systemStatus.currentState = newState;
        xSemaphoreGive(xMutexStatus);
    }
}

/*
 * Send a sensor data item to the ESP32 via the sensor data queue.
 */
static void enqueue_sensor_data(SensorDataType_t type, uint16_t value)
{
    SensorData_t data = {
        .type      = type,
        .value     = value,
        .timestamp = xTaskGetTickCount()
    };
    xQueueSend(xQueueSensorData, &data, 0);  /* Non-blocking */
}

/*
 * Send an actuator command to the actuator control task.
 */
static void enqueue_actuator_cmd(ActuatorCmdType_t type, uint16_t value,
                                  uint16_t duration)
{
    ActuatorCmd_t cmd = {
        .type     = type,
        .value    = value,
        .duration = duration
    };
    xQueueSend(xQueueActuatorCmd, &cmd, pdMS_TO_TICKS(10));
}

/*
 * Send a protocol packet to ESP32 (mutex-protected).
 */
static void send_packet_to_esp(uint8_t type, const uint8_t *payload,
                                uint8_t payloadLen)
{
    uint8_t buf[PROTO_MAX_PACKET_SIZE];
    uint8_t len = protocol_build_packet(buf, type, payload, payloadLen);

    if (xSemaphoreTake(xMutexUART, pdMS_TO_TICKS(50)) == pdTRUE) {
        HAL_UART_Send(buf, len);
        xSemaphoreGive(xMutexUART);
    }
}

/*
 * Send an event code to ESP32.
 */
static void send_event(EventCode_t event)
{
    uint8_t payload = (uint8_t)event;
    send_packet_to_esp(MSG_EVENT, &payload, 1);
}

/*
 * Send current FSM state to ESP32.
 */
static void send_state_update(SystemState_t state)
{
    uint8_t payload = (uint8_t)state;
    send_packet_to_esp(MSG_STATE, &payload, 1);
}


/* ========================= ULTRASONIC TIMER CALLBACK ===================== */

/*
 * Periodic timer callback to trigger ultrasonic measurements.
 * Fires every 200ms to continuously monitor pet distance.
 */
static void vUltrasonicTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    HAL_Ultrasonic_Trigger();
}


/* ========================= TASK: PET MONITOR ============================= */
/*
 * Highest priority task. The "brain" of the system.
 *
 * Waits on xSemProximity and xSemInteraction to process events.
 * Manages the FSM state transitions.
 * Enqueues actuator commands based on current state and sensor data.
 */
static void tskPetMonitor(void *pvParameters)
{
    (void)pvParameters;
    TickType_t lastWakeTime = xTaskGetTickCount();
    SystemState_t prevState = STATE_IDLE;

    for (;;) {
        /* ---- Check ultrasonic proximity semaphore ---- */
        if (xSemaphoreTake(xSemProximity, pdMS_TO_TICKS(100)) == pdTRUE) {
            uint16_t dist = HAL_Ultrasonic_GetDistance_cm();

            /* Update status */
            if (xSemaphoreTake(xMutexStatus, pdMS_TO_TICKS(10)) == pdTRUE) {
                systemStatus.lastDistance_cm = dist;
                xSemaphoreGive(xMutexStatus);
            }

            /* Send distance to ESP32 periodically */
            enqueue_sensor_data(SENSOR_DISTANCE, dist);

            /* --- FSM logic based on distance --- */
            switch (systemStatus.currentState) {

            case STATE_IDLE:
                if (dist < ULTRASONIC_PET_NEAR_CM) {
                    /* Pet has arrived */
                    status_update_state(STATE_PET_DETECTED);
                    send_event(EVT_PET_ARRIVED);
                    send_state_update(STATE_PET_DETECTED);

                    if (xSemaphoreTake(xMutexStatus, pdMS_TO_TICKS(10)) == pdTRUE) {
                        systemStatus.petPresent = 1;
                        systemStatus.lastPetSeenTick = xTaskGetTickCount();
                        systemStatus.visitCount++;
                        xSemaphoreGive(xMutexStatus);
                    }

                    /* Acknowledge with a short chirp */
                    enqueue_actuator_cmd(ACT_BUZZER_TONE, TONE_INTERACTION_ACK, 100);
                }
                break;

            case STATE_PET_DETECTED:
                if (dist < ULTRASONIC_PET_NEAR_CM) {
                    /* Pet still here — update last seen time */
                    if (xSemaphoreTake(xMutexStatus, pdMS_TO_TICKS(10)) == pdTRUE) {
                        systemStatus.lastPetSeenTick = xTaskGetTickCount();
                        xSemaphoreGive(xMutexStatus);
                    }
                } else if (dist > ULTRASONIC_PET_FAR_CM) {
                    /* Check if pet has been gone long enough */
                    TickType_t elapsed = xTaskGetTickCount() - systemStatus.lastPetSeenTick;
                    if (elapsed > pdMS_TO_TICKS(PET_TIMEOUT_MS)) {
                        status_update_state(STATE_IDLE);
                        send_event(EVT_PET_LEFT);
                        send_state_update(STATE_IDLE);

                        if (xSemaphoreTake(xMutexStatus, pdMS_TO_TICKS(10)) == pdTRUE) {
                            systemStatus.petPresent = 0;
                            xSemaphoreGive(xMutexStatus);
                        }
                    }
                }
                break;

            case STATE_FEEDING:
                /* Monitor if pet walks away during feeding */
                if (dist < ULTRASONIC_PET_NEAR_CM) {
                    if (xSemaphoreTake(xMutexStatus, pdMS_TO_TICKS(10)) == pdTRUE) {
                        systemStatus.lastPetSeenTick = xTaskGetTickCount();
                        xSemaphoreGive(xMutexStatus);
                    }
                }
                /* Check feeding timeout */
                {
                    TickType_t feedElapsed = xTaskGetTickCount() - systemStatus.feedingStartTick;
                    if (feedElapsed > pdMS_TO_TICKS(FEED_TIMEOUT_MS)) {
                        /* Feeding timeout — close gate, return to PET_DETECTED */
                        enqueue_actuator_cmd(ACT_FEED_CLOSE, 0, 0);
                        status_update_state(STATE_PET_DETECTED);
                        send_event(EVT_FEED_DONE);
                        send_state_update(STATE_PET_DETECTED);

                        if (xSemaphoreTake(xMutexStatus, pdMS_TO_TICKS(10)) == pdTRUE) {
                            systemStatus.feedCount++;
                            xSemaphoreGive(xMutexStatus);
                        }
                    }
                }
                break;

            case STATE_PLAYING:
                /* Track pet engagement via distance changes */
                if (dist < ULTRASONIC_PET_NEAR_CM) {
                    if (xSemaphoreTake(xMutexStatus, pdMS_TO_TICKS(10)) == pdTRUE) {
                        systemStatus.lastPetSeenTick = xTaskGetTickCount();
                        xSemaphoreGive(xMutexStatus);
                    }
                }
                /* Check play duration */
                {
                    TickType_t playElapsed = xTaskGetTickCount() - systemStatus.playingStartTick;
                    if (playElapsed > pdMS_TO_TICKS(PLAY_DURATION_MS)) {
                        enqueue_actuator_cmd(ACT_PLAY_STOP, 0, 0);
                        status_update_state(STATE_PET_DETECTED);
                        send_event(EVT_PLAY_DONE);
                        send_state_update(STATE_PET_DETECTED);

                        if (xSemaphoreTake(xMutexStatus, pdMS_TO_TICKS(10)) == pdTRUE) {
                            systemStatus.playCount++;
                            xSemaphoreGive(xMutexStatus);
                        }
                    }
                }
                break;

            case STATE_ALERT:
                /* Stay in alert until water is refilled or condition clears */
                /* (Checked in tskSensorPoll) */
                break;

            case STATE_INTERACTION:
                /* Transient state — will be handled below by shock semaphore */
                break;

            default:
                break;
            }
        }

        /* ---- Check shock/interaction semaphore (non-blocking) ---- */
        if (xSemaphoreTake(xSemInteraction, 0) == pdTRUE) {
            send_event(EVT_INTERACTION);
            enqueue_sensor_data(SENSOR_SHOCK_EVENT, 1);

            if (xSemaphoreTake(xMutexStatus, pdMS_TO_TICKS(10)) == pdTRUE) {
                systemStatus.interactionCount++;
                systemStatus.lastPetSeenTick = xTaskGetTickCount();
                xSemaphoreGive(xMutexStatus);
            }

            switch (systemStatus.currentState) {

            case STATE_PET_DETECTED:
                /* Pet is nudging — acknowledge and possibly trigger play */
                enqueue_actuator_cmd(ACT_BUZZER_TONE, TONE_INTERACTION_ACK, 150);
                break;

            case STATE_FEEDING:
                /* Vibration during feeding = pet is actively eating */
                /* Just log it, don't change state */
                break;

            case STATE_PLAYING:
                /* Pet is engaged with the station during play — good sign */
                enqueue_actuator_cmd(ACT_BUZZER_TONE, TONE_INTERACTION_ACK, 80);
                break;

            case STATE_IDLE:
                /* Pet nudging while "not detected" — it arrived! */
                status_update_state(STATE_PET_DETECTED);
                send_event(EVT_PET_ARRIVED);
                send_state_update(STATE_PET_DETECTED);

                if (xSemaphoreTake(xMutexStatus, pdMS_TO_TICKS(10)) == pdTRUE) {
                    systemStatus.petPresent = 1;
                    systemStatus.visitCount++;
                    xSemaphoreGive(xMutexStatus);
                }

                enqueue_actuator_cmd(ACT_BUZZER_TONE, TONE_INTERACTION_ACK, 150);
                break;

            default:
                break;
            }
        }

        /* ---- Check water level alert condition ---- */
        if (systemStatus.waterLow &&
            systemStatus.currentState != STATE_ALERT &&
            systemStatus.currentState != STATE_FEEDING &&
            systemStatus.currentState != STATE_PLAYING) {
            status_update_state(STATE_ALERT);
            send_event(EVT_WATER_LOW);
            send_state_update(STATE_ALERT);
            enqueue_actuator_cmd(ACT_BUZZER_TONE, TONE_WATER_LOW, 2000);
        }

        /* Small yield to avoid starving lower-priority tasks */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


/* ========================= TASK: ACTUATOR CONTROL ======================== */
/*
 * Waits on xQueueActuatorCmd and executes actuator commands.
 * Owns all actuator hardware: servo, buzzer, laser.
 */
static void tskActuatorControl(void *pvParameters)
{
    (void)pvParameters;
    ActuatorCmd_t cmd;

    /* Servo sweep state for play mode */
    bool     sweepActive    = false;
    uint16_t sweepPosition  = SERVO_PLAY_CENTER;
    int16_t  sweepDirection = 50;  /* Step size in us per iteration */

    for (;;) {
        /* Wait for a command with timeout (allows sweep update during play) */
        if (xQueueReceive(xQueueActuatorCmd, &cmd, pdMS_TO_TICKS(50)) == pdTRUE) {

            switch (cmd.type) {

            case ACT_BUZZER_TONE:
                HAL_Buzzer_SetTone(cmd.value);
                if (cmd.duration > 0) {
                    vTaskDelay(pdMS_TO_TICKS(cmd.duration));
                    HAL_Buzzer_Off();
                }
                break;

            case ACT_BUZZER_OFF:
                HAL_Buzzer_Off();
                break;

            case ACT_SERVO_POSITION:
                HAL_Servo_SetPosition(cmd.value);
                break;

            case ACT_SERVO_SWEEP:
                sweepActive = true;
                break;

            case ACT_SERVO_STOP:
                sweepActive = false;
                HAL_Servo_SetPosition(SERVO_PLAY_CENTER);
                break;

            case ACT_LASER_ON:
                HAL_Laser_On();
                break;

            case ACT_LASER_OFF:
                HAL_Laser_Off();
                break;

            case ACT_FEED_OPEN:
                /* Stop any sweep, move servo to food gate position */
                sweepActive = false;
                HAL_Laser_Off();
                HAL_Servo_SetPosition(SERVO_FEED_OPEN);
                /* Chirp to call pet to eat */
                HAL_Buzzer_SetTone(TONE_FOOD_READY);
                vTaskDelay(pdMS_TO_TICKS(500));
                HAL_Buzzer_Off();
                break;

            case ACT_FEED_CLOSE:
                HAL_Servo_SetPosition(SERVO_FEED_CLOSED);
                break;

            case ACT_PLAY_START:
                /* Activate laser and start sweep */
                HAL_Laser_On();
                sweepActive   = true;
                sweepPosition = SERVO_PLAY_CENTER;
                break;

            case ACT_PLAY_STOP:
                /* Deactivate everything */
                sweepActive = false;
                HAL_Laser_Off();
                HAL_Servo_SetPosition(SERVO_PLAY_CENTER);
                break;

            case ACT_ALL_STOP:
                /* Emergency stop */
                sweepActive = false;
                HAL_Laser_Off();
                HAL_Buzzer_Off();
                HAL_Servo_SetPosition(SERVO_FEED_CLOSED);
                break;

            default:
                break;
            }
        }

        /* --- Continuous servo sweep during play mode --- */
        if (sweepActive) {
            sweepPosition += sweepDirection;

            /* Reverse direction at limits */
            if (sweepPosition >= SERVO_PLAY_RIGHT) {
                sweepPosition = SERVO_PLAY_RIGHT;
                sweepDirection = -sweepDirection;
            } else if (sweepPosition <= SERVO_PLAY_LEFT) {
                sweepPosition = SERVO_PLAY_LEFT;
                sweepDirection = -sweepDirection;
            }

            HAL_Servo_SetPosition(sweepPosition);
        }
    }
}


/* ========================= TASK: SENSOR POLL ============================= */
/*
 * Periodically reads the water level sensor via ADC.
 * Applies a simple moving average filter.
 * Pushes data to xQueueSensorData for cloud upload.
 * Triggers water low alerts.
 */
static void tskSensorPoll(void *pvParameters)
{
    (void)pvParameters;
    TickType_t lastWakeTime = xTaskGetTickCount();

    /* Moving average filter (4 samples) */
    #define WATER_AVG_SAMPLES   4
    uint16_t waterSamples[WATER_AVG_SAMPLES] = {0};
    uint8_t  sampleIdx = 0;
    uint32_t waterSum   = 0;
    bool     bufferFull = false;

    for (;;) {
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SENSOR_POLL_INTERVAL_MS));

        /* Read ADC */
        uint16_t rawValue = HAL_WaterLevel_Read();

        /* Update moving average */
        waterSum -= waterSamples[sampleIdx];
        waterSamples[sampleIdx] = rawValue;
        waterSum += rawValue;
        sampleIdx = (sampleIdx + 1) % WATER_AVG_SAMPLES;
        if (sampleIdx == 0) bufferFull = true;

        uint16_t avgValue = (uint16_t)(waterSum /
                            (bufferFull ? WATER_AVG_SAMPLES : (sampleIdx ? sampleIdx : 1)));

        /* Update system status */
        if (xSemaphoreTake(xMutexStatus, pdMS_TO_TICKS(10)) == pdTRUE) {
            systemStatus.lastWaterLevel = avgValue;

            if (avgValue < WATER_LEVEL_CRIT_THRESHOLD) {
                systemStatus.waterLow = 2;  /* Critical */
            } else if (avgValue < WATER_LEVEL_LOW_THRESHOLD) {
                systemStatus.waterLow = 1;  /* Low */
            } else {
                /* Water level OK — if we were in ALERT, return to IDLE */
                if (systemStatus.waterLow && systemStatus.currentState == STATE_ALERT) {
                    systemStatus.currentState = STATE_IDLE;
                    send_state_update(STATE_IDLE);
                }
                systemStatus.waterLow = 0;
            }

            xSemaphoreGive(xMutexStatus);
        }

        /* Push to sensor data queue for ESP32 */
        enqueue_sensor_data(SENSOR_WATER_LEVEL, avgValue);
    }
}


/* ========================= TASK: UART COMMUNICATION ====================== */
/*
 * Handles bi-directional packetized UART with ESP32.
 *
 * TX: Dequeues from xQueueSensorData, builds packets, sends to ESP32.
 * RX: Reads UART ring buffer, feeds parser, processes received commands.
 */
static void tskUARTComm(void *pvParameters)
{
    (void)pvParameters;
    SensorData_t sensorData;

    protocol_parser_init(&uartParser);

    for (;;) {
        /* ---- TX: Send queued sensor data to ESP32 ---- */
        while (xQueueReceive(xQueueSensorData, &sensorData, 0) == pdTRUE) {
            uint8_t payload[4];

            switch (sensorData.type) {
            case SENSOR_DISTANCE:
                payload[0] = (sensorData.value >> 8) & 0xFF;
                payload[1] = sensorData.value & 0xFF;
                send_packet_to_esp(MSG_DISTANCE, payload, 2);
                break;

            case SENSOR_WATER_LEVEL:
                payload[0] = (sensorData.value >> 8) & 0xFF;
                payload[1] = sensorData.value & 0xFF;
                send_packet_to_esp(MSG_WATER_LEVEL, payload, 2);
                break;

            case SENSOR_SHOCK_EVENT:
                payload[0] = (uint8_t)sensorData.value;
                send_packet_to_esp(MSG_EVENT, payload, 1);
                break;

            case SENSOR_STATE_CHANGE:
                payload[0] = (uint8_t)sensorData.value;
                send_packet_to_esp(MSG_STATE, payload, 1);
                break;
            }
        }

        /* ---- RX: Process incoming bytes from ESP32 ---- */
        uint8_t byte;
        while (HAL_UART_ReadByte(&byte)) {
            if (protocol_parser_feed(&uartParser, byte)) {
                /* Complete valid packet received — process command */
                Packet_t *pkt = &uartParser.packet;

                switch (pkt->type) {

                case MSG_CMD_FEED:
                    /* Remote feed command */
                    if (systemStatus.currentState == STATE_PET_DETECTED ||
                        systemStatus.currentState == STATE_IDLE) {

                        status_update_state(STATE_FEEDING);
                        send_state_update(STATE_FEEDING);
                        send_event(EVT_FEED_START);

                        if (xSemaphoreTake(xMutexStatus, pdMS_TO_TICKS(10)) == pdTRUE) {
                            systemStatus.feedingStartTick = xTaskGetTickCount();
                            xSemaphoreGive(xMutexStatus);
                        }

                        enqueue_actuator_cmd(ACT_FEED_OPEN, 0, 0);
                    }
                    break;

                case MSG_CMD_PLAY:
                    /* Remote play command. payload[0] = pattern (reserved) */
                    if (systemStatus.currentState == STATE_PET_DETECTED ||
                        systemStatus.currentState == STATE_IDLE) {

                        status_update_state(STATE_PLAYING);
                        send_state_update(STATE_PLAYING);
                        send_event(EVT_PLAY_START);

                        if (xSemaphoreTake(xMutexStatus, pdMS_TO_TICKS(10)) == pdTRUE) {
                            systemStatus.playingStartTick = xTaskGetTickCount();
                            xSemaphoreGive(xMutexStatus);
                        }

                        enqueue_actuator_cmd(ACT_PLAY_START, 0, 0);
                    }
                    break;

                case MSG_CMD_STOP:
                    /* Emergency stop */
                    enqueue_actuator_cmd(ACT_ALL_STOP, 0, 0);
                    status_update_state(STATE_IDLE);
                    send_state_update(STATE_IDLE);
                    break;

                case MSG_CMD_CALL_PET:
                    /* Play call melody */
                    enqueue_actuator_cmd(ACT_BUZZER_TONE, TONE_CALL_PET, 1000);
                    break;

                case MSG_CMD_SET_SCHEDULE:
                    /* Update feed schedule: slot, hour, minute, enabled */
                    if (pkt->payloadLen >= 4 && pkt->payload[0] < MAX_FEED_SCHEDULES) {
                        uint8_t slot = pkt->payload[0];
                        feedSchedule[slot].hour    = pkt->payload[1];
                        feedSchedule[slot].minute  = pkt->payload[2];
                        feedSchedule[slot].enabled = pkt->payload[3];
                    }
                    break;

                case MSG_CMD_SET_THRESHOLD:
                    /* Update threshold: param_id, value_hi, value_lo */
                    /* Reserved for future: adjustable water/proximity thresholds */
                    break;

                case MSG_CMD_ESP_STATUS:
                    /* ESP32 sends its sensor readings (temp, humidity) */
                    /* We just acknowledge — ESP32 handles cloud upload of DHT */
                    {
                        uint8_t ack = pkt->seq;
                        send_packet_to_esp(MSG_ACK, &ack, 1);
                    }
                    break;

                default:
                    break;
                }
            }
        }

        /* Yield between TX/RX cycles */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


/* ========================= TASK: SCHEDULER =============================== */
/*
 * Low priority task that manages feeding and auto-play schedules.
 *
 * NOTE: The MCXC444 does not have an RTC. Time-of-day scheduling requires
 * the ESP32 to send the current time periodically, or you can use a
 * simple tick-based interval system.
 *
 * For demonstration, this uses a tick-based approach:
 *   - Feed interval: configurable (default every 8 hours worth of ticks)
 *   - Auto-play: triggered when pet has been present for a while without play
 *
 * For production, the ESP32 should send MSG_CMD_FEED at the right time
 * based on its NTP-synced clock, and this task can handle auto-play logic.
 */
static void tskScheduler(void *pvParameters)
{
    (void)pvParameters;
    TickType_t lastFeedTick = 0;
    TickType_t lastPlayTick = 0;

    /* Minimum intervals between auto-triggers */
    const TickType_t minFeedInterval = pdMS_TO_TICKS(4UL * 3600UL * 1000UL); /* 4 hours */
    const TickType_t minPlayInterval = pdMS_TO_TICKS(2UL * 3600UL * 1000UL); /* 2 hours */
    const TickType_t autoPlayDelay   = pdMS_TO_TICKS(60000UL);  /* Pet present 60s → auto play */

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));  /* Check every 5 seconds */

        TickType_t now = xTaskGetTickCount();
        SystemState_t state = systemStatus.currentState;

        /* ---- Auto-play logic ---- */
        /*
         * If pet has been in PET_DETECTED state for autoPlayDelay
         * and hasn't played recently, trigger play mode.
         */
        if (state == STATE_PET_DETECTED &&
            systemStatus.petPresent &&
            (now - lastPlayTick) > minPlayInterval) {

            TickType_t presentDuration = now - systemStatus.lastPetSeenTick;

            /* Only auto-play if pet arrived recently and has been staying */
            if (presentDuration < autoPlayDelay &&
                (now - systemStatus.lastPetSeenTick) > pdMS_TO_TICKS(5000)) {

                /* Trigger play mode */
                status_update_state(STATE_PLAYING);
                send_state_update(STATE_PLAYING);
                send_event(EVT_PLAY_START);

                if (xSemaphoreTake(xMutexStatus, pdMS_TO_TICKS(10)) == pdTRUE) {
                    systemStatus.playingStartTick = xTaskGetTickCount();
                    xSemaphoreGive(xMutexStatus);
                }

                enqueue_actuator_cmd(ACT_PLAY_START, 0, 0);
                lastPlayTick = now;
            }
        }

        /*
         * Feed schedule is primarily driven by ESP32 sending MSG_CMD_FEED
         * at the correct time (since ESP32 has NTP time).
         *
         * This task serves as a backup / interval-based fallback.
         */
        if (state == STATE_PET_DETECTED &&
            systemStatus.petPresent &&
            (now - lastFeedTick) > minFeedInterval) {

            /* Don't auto-feed — wait for ESP32 schedule command */
            /* This check is here as a framework for future tick-based feeding */
        }
    }
}


/* ========================= MAIN ========================================== */

int main(void)
{
    /* Initialize hardware */
    HAL_Init();

    /* Create synchronization primitives */
    xSemProximity    = xSemaphoreCreateBinary();
    xSemInteraction  = xSemaphoreCreateBinary();
    xQueueActuatorCmd = xQueueCreate(ACTUATOR_QUEUE_SIZE, sizeof(ActuatorCmd_t));
    xQueueSensorData  = xQueueCreate(SENSOR_QUEUE_SIZE, sizeof(SensorData_t));
    xMutexUART       = xSemaphoreCreateMutex();
    xMutexStatus     = xSemaphoreCreateMutex();

    /* Verify all created successfully */
    configASSERT(xSemProximity);
    configASSERT(xSemInteraction);
    configASSERT(xQueueActuatorCmd);
    configASSERT(xQueueSensorData);
    configASSERT(xMutexUART);
    configASSERT(xMutexStatus);

    /* Create tasks */
    xTaskCreate(tskPetMonitor,      "PetMon",   STACK_SIZE_PET_MONITOR,
                NULL, PRIORITY_PET_MONITOR,    &hTaskPetMonitor);

    xTaskCreate(tskActuatorControl, "ActCtrl",  STACK_SIZE_ACTUATOR_CTRL,
                NULL, PRIORITY_ACTUATOR_CTRL,  &hTaskActuatorCtrl);

    xTaskCreate(tskSensorPoll,      "SensPoll", STACK_SIZE_SENSOR_POLL,
                NULL, PRIORITY_SENSOR_POLL,    &hTaskSensorPoll);

    xTaskCreate(tskUARTComm,        "UARTComm", STACK_SIZE_UART_COMM,
                NULL, PRIORITY_UART_COMM,      &hTaskUARTComm);

    xTaskCreate(tskScheduler,       "Sched",    STACK_SIZE_SCHEDULER,
                NULL, PRIORITY_SCHEDULER,      &hTaskScheduler);

    /* Create periodic ultrasonic trigger timer (200ms interval) */
    hTimerUltrasonic = xTimerCreate("UltraTrig", pdMS_TO_TICKS(200),
                                     pdTRUE,  /* Auto-reload */
                                     NULL,
                                     vUltrasonicTimerCallback);
    xTimerStart(hTimerUltrasonic, 0);

    /* Start the scheduler — this should never return */
    vTaskStartScheduler();

    /* Should not reach here */
    for (;;) {}

    return 0;
}
