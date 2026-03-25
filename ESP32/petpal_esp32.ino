/*
 * =============================================================================
 * PetPal - ESP32 Firmware (Arduino IDE)
 * =============================================================================
 * File:    petpal_esp32.ino
 * Desc:    Wi-Fi gateway, Firebase RTDB, OpenAI API, DHT11 sensor,
 *          and bi-directional UART communication with MCXC444.
 *
 * Required Libraries (install via Arduino Library Manager):
 *   - DHT sensor library (by Adafruit)
 *   - ArduinoJson (by Benoit Blanchon)
 *   - Firebase ESP Client (by Mobizt) — "Firebase-ESP-Client"
 *   - WiFi (built-in ESP32)
 *   - HTTPClient (built-in ESP32)
 *
 * Hardware connections:
 *   DHT11 data  → GPIO 4
 *   UART TX     → GPIO 17 (Serial2 TX) → MCXC444 RX
 *   UART RX     → GPIO 16 (Serial2 RX) → MCXC444 TX
 *   (Shared GND between ESP32 and MCXC444)
 *
 * IMPORTANT: Replace the placeholder credentials below with your own:
 *   - WiFi SSID and password
 *   - Firebase project URL and API key
 *   - OpenAI API key
 * =============================================================================
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <Firebase_ESP_Client.h>
#include <time.h>

/* Provide Firebase token generation helpers */
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"


/* ========================= CONFIGURATION ================================= */

/* --- WiFi --- */
#define WIFI_SSID           "YOUR_HOTSPOT_SSID"
#define WIFI_PASSWORD       "YOUR_HOTSPOT_PASSWORD"

/* --- Firebase --- */
#define FIREBASE_HOST       "YOUR_PROJECT.firebaseio.com"
#define FIREBASE_API_KEY    "YOUR_FIREBASE_API_KEY"
/* For anonymous auth or email auth — adjust as needed */
#define FIREBASE_USER_EMAIL ""
#define FIREBASE_USER_PASS  ""

/* --- OpenAI --- */
#define OPENAI_API_KEY      "YOUR_OPENAI_API_KEY"
#define OPENAI_MODEL        "gpt-4o-mini"
#define OPENAI_ENDPOINT     "https://api.openai.com/v1/chat/completions"

/* --- DHT11 Sensor --- */
#define DHT_PIN             4
#define DHT_TYPE            DHT11

/* --- UART to MCXC444 --- */
#define UART_BAUD           115200
#define UART_RX_PIN         16
#define UART_TX_PIN         17

/* --- Timing Intervals (ms) --- */
#define DHT_READ_INTERVAL       5000        /* Read DHT every 5 seconds */
#define FIREBASE_PUSH_INTERVAL  10000       /* Push sensor data every 10s */
#define FIREBASE_CMD_POLL       3000        /* Poll for commands every 3s */
#define AI_REPORT_INTERVAL      (3600000UL * 12)  /* Auto AI report every 12h */
#define STATUS_SEND_INTERVAL    5000        /* Send DHT data to MCXC444 every 5s */

/* --- Pet thresholds --- */
#define TEMP_HIGH_ALERT     33.0
#define TEMP_LOW_ALERT      18.0


/* ========================= PROTOCOL DEFINITIONS ========================== */
/*
 * Must match protocol.h on the MCXC444 side.
 */

#define PROTO_START_BYTE    0xAA
#define PROTO_MAX_PAYLOAD   16

/* Message types: MCXC444 → ESP32 */
#define MSG_DISTANCE        0x01
#define MSG_WATER_LEVEL     0x02
#define MSG_EVENT           0x03
#define MSG_STATE           0x04
#define MSG_STATUS          0x05
#define MSG_ACK             0x06

/* Message types: ESP32 → MCXC444 */
#define MSG_CMD_FEED        0x10
#define MSG_CMD_PLAY        0x11
#define MSG_CMD_STOP        0x12
#define MSG_CMD_CALL_PET    0x13
#define MSG_CMD_SET_SCHED   0x14
#define MSG_CMD_SET_THRESH  0x15
#define MSG_CMD_ESP_STATUS  0x16
#define MSG_CMD_ACK         0x17

/* Event codes */
#define EVT_PET_ARRIVED     0x01
#define EVT_PET_LEFT        0x02
#define EVT_FEED_START      0x03
#define EVT_FEED_DONE       0x04
#define EVT_PLAY_START      0x05
#define EVT_PLAY_DONE       0x06
#define EVT_INTERACTION     0x07
#define EVT_WATER_LOW       0x08
#define EVT_WATER_CRIT      0x09
#define EVT_TEMP_ALERT      0x0A

/* FSM states */
#define STATE_IDLE          0
#define STATE_PET_DETECTED  1
#define STATE_FEEDING       2
#define STATE_PLAYING       3
#define STATE_INTERACTION   4
#define STATE_ALERT         5


/* ========================= GLOBAL OBJECTS ================================ */

DHT dht(DHT_PIN, DHT_TYPE);

FirebaseData fbData;
FirebaseAuth fbAuth;
FirebaseConfig fbConfig;

/* Parser state machine */
enum ParseState { WAIT_START, WAIT_LENGTH, WAIT_DATA, WAIT_CRC };
ParseState parseState = WAIT_START;
uint8_t pktBuffer[PROTO_MAX_PAYLOAD + 4];
uint8_t pktDataIdx = 0;
uint8_t pktDataExpected = 0;
uint8_t pktSeq = 0;
uint8_t pktType = 0;
uint8_t pktPayload[PROTO_MAX_PAYLOAD];
uint8_t pktPayloadLen = 0;

/* Sequence counter for outgoing packets */
uint8_t txSeqCounter = 0;

/* Latest sensor values */
float   lastTemp        = 0.0;
float   lastHumidity    = 0.0;
uint16_t lastDistance    = 999;
uint16_t lastWaterLevel  = 4095;
uint8_t  currentFSMState = STATE_IDLE;
bool     petPresent      = false;

/* Daily event counters for AI report */
uint32_t dailyVisits       = 0;
uint32_t dailyFeeds        = 0;
uint32_t dailyPlaySessions = 0;
uint32_t dailyInteractions = 0;
uint32_t dailyWaterAlerts  = 0;
String   eventLog          = "";

/* Timing trackers */
unsigned long lastDHTRead       = 0;
unsigned long lastFirebasePush  = 0;
unsigned long lastFirebasePoll  = 0;
unsigned long lastAIReport      = 0;
unsigned long lastStatusSend    = 0;

/* Firebase ready flag */
bool firebaseReady = false;


/* ========================= CRC-8 IMPLEMENTATION ========================== */

uint8_t crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc = crc << 1;
        }
    }
    return crc;
}


/* ========================= PACKET TX ===================================== */

void sendPacket(uint8_t type, const uint8_t *payload, uint8_t payloadLen) {
    if (payloadLen > PROTO_MAX_PAYLOAD) payloadLen = PROTO_MAX_PAYLOAD;

    uint8_t seq = txSeqCounter++;
    uint8_t length = 2 + payloadLen;  /* SEQ + TYPE + payload */

    uint8_t buf[PROTO_MAX_PAYLOAD + 5];
    buf[0] = PROTO_START_BYTE;
    buf[1] = length;
    buf[2] = seq;
    buf[3] = type;

    if (payload != NULL && payloadLen > 0) {
        memcpy(&buf[4], payload, payloadLen);
    }

    /* CRC over SEQ + TYPE + PAYLOAD */
    buf[4 + payloadLen] = crc8(&buf[2], length);

    Serial2.write(buf, 4 + payloadLen + 1);
}

/* Convenience: send a command with no payload */
void sendCommand(uint8_t type) {
    sendPacket(type, NULL, 0);
}


/* ========================= PACKET RX (PARSER) ============================ */

/*
 * Feed one byte into the packet parser.
 * Returns true when a complete valid packet is received.
 * Parsed data is in pktType, pktPayload[], pktPayloadLen.
 */
bool parseByte(uint8_t byte) {
    switch (parseState) {

    case WAIT_START:
        if (byte == PROTO_START_BYTE) {
            parseState = WAIT_LENGTH;
        }
        break;

    case WAIT_LENGTH:
        if (byte < 2 || byte > (2 + PROTO_MAX_PAYLOAD)) {
            parseState = WAIT_START;
        } else {
            pktDataExpected = byte;
            pktDataIdx = 0;
            parseState = WAIT_DATA;
        }
        break;

    case WAIT_DATA:
        pktBuffer[pktDataIdx++] = byte;
        if (pktDataIdx >= pktDataExpected) {
            pktSeq = pktBuffer[0];
            pktType = pktBuffer[1];
            pktPayloadLen = pktDataExpected - 2;
            if (pktPayloadLen > 0) {
                memcpy(pktPayload, &pktBuffer[2], pktPayloadLen);
            }
            parseState = WAIT_CRC;
        }
        break;

    case WAIT_CRC:
        {
            uint8_t computed = crc8(pktBuffer, pktDataExpected);
            parseState = WAIT_START;
            if (computed == byte) {
                return true;  /* Valid packet! */
            }
        }
        break;
    }
    return false;
}


/* ========================= PROCESS RECEIVED PACKET ======================= */

void processPacket() {
    switch (pktType) {

    case MSG_DISTANCE:
        if (pktPayloadLen >= 2) {
            lastDistance = (pktPayload[0] << 8) | pktPayload[1];
        }
        break;

    case MSG_WATER_LEVEL:
        if (pktPayloadLen >= 2) {
            lastWaterLevel = (pktPayload[0] << 8) | pktPayload[1];
        }
        break;

    case MSG_EVENT:
        if (pktPayloadLen >= 1) {
            uint8_t eventCode = pktPayload[0];
            handleEvent(eventCode);
        }
        break;

    case MSG_STATE:
        if (pktPayloadLen >= 1) {
            currentFSMState = pktPayload[0];
            petPresent = (currentFSMState != STATE_IDLE);
        }
        break;

    case MSG_ACK:
        /* Acknowledgement from MCXC444 — no action needed */
        break;

    default:
        break;
    }
}


/* ========================= EVENT HANDLING ================================ */

String eventCodeToString(uint8_t code) {
    switch (code) {
        case EVT_PET_ARRIVED: return "pet_arrived";
        case EVT_PET_LEFT:    return "pet_left";
        case EVT_FEED_START:  return "feed_start";
        case EVT_FEED_DONE:   return "feed_done";
        case EVT_PLAY_START:  return "play_start";
        case EVT_PLAY_DONE:   return "play_done";
        case EVT_INTERACTION: return "interaction";
        case EVT_WATER_LOW:   return "water_low";
        case EVT_WATER_CRIT:  return "water_critical";
        case EVT_TEMP_ALERT:  return "temp_alert";
        default:              return "unknown";
    }
}

String stateToString(uint8_t state) {
    switch (state) {
        case STATE_IDLE:         return "IDLE";
        case STATE_PET_DETECTED: return "PET_DETECTED";
        case STATE_FEEDING:      return "FEEDING";
        case STATE_PLAYING:      return "PLAYING";
        case STATE_INTERACTION:  return "INTERACTION";
        case STATE_ALERT:        return "ALERT";
        default:                 return "UNKNOWN";
    }
}

void handleEvent(uint8_t eventCode) {
    String eventName = eventCodeToString(eventCode);
    Serial.println("[EVENT] " + eventName);

    /* Update daily counters */
    switch (eventCode) {
        case EVT_PET_ARRIVED: dailyVisits++; break;
        case EVT_FEED_START:  dailyFeeds++; break;
        case EVT_PLAY_START:  dailyPlaySessions++; break;
        case EVT_INTERACTION: dailyInteractions++; break;
        case EVT_WATER_LOW:
        case EVT_WATER_CRIT:  dailyWaterAlerts++; break;
    }

    /* Append to event log for AI report */
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
        char timeStr[16];
        strftime(timeStr, sizeof(timeStr), "%H:%M:%S", &timeinfo);
        eventLog += timeStr;
        eventLog += " - ";
        eventLog += eventName;
        eventLog += "\n";
    }

    /* Push event to Firebase */
    if (firebaseReady) {
        String path = "petpal/events/";
        FirebaseJson json;
        json.set("type", eventName);
        json.set("timestamp/.sv", "timestamp");
        json.set("distance", lastDistance);
        json.set("water_level", lastWaterLevel);
        json.set("temp", lastTemp);
        json.set("humidity", lastHumidity);

        Firebase.RTDB.pushJSON(&fbData, path.c_str(), &json);
    }
}


/* ========================= WIFI SETUP ==================================== */

void setupWiFi() {
    Serial.print("[WIFI] Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print("[WIFI] Connected! IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println();
        Serial.println("[WIFI] Failed to connect. Will retry in loop.");
    }
}


/* ========================= NTP TIME SYNC ================================= */

void setupTime() {
    configTime(8 * 3600, 0, "pool.ntp.org");  /* UTC+8 for Singapore */
    Serial.print("[TIME] Syncing NTP...");

    struct tm timeinfo;
    int attempts = 0;
    while (!getLocalTime(&timeinfo) && attempts < 10) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (getLocalTime(&timeinfo)) {
        Serial.println(" OK");
        Serial.printf("[TIME] Current time: %02d:%02d:%02d\n",
                      timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
        Serial.println(" Failed (will use millis)");
    }
}


/* ========================= FIREBASE SETUP ================================ */

void setupFirebase() {
    Serial.println("[FIREBASE] Initializing...");

    fbConfig.api_key = FIREBASE_API_KEY;
    fbConfig.database_url = FIREBASE_HOST;

    /* Anonymous sign-in (simplest for prototyping) */
    /* For email/password auth, set fbAuth.user.email and .password */
    if (strlen(FIREBASE_USER_EMAIL) > 0) {
        fbAuth.user.email = FIREBASE_USER_EMAIL;
        fbAuth.user.password = FIREBASE_USER_PASS;
    }

    fbConfig.token_status_callback = tokenStatusCallback;

    Firebase.begin(&fbConfig, &fbAuth);
    Firebase.reconnectWiFi(true);

    /* Wait for token */
    unsigned long start = millis();
    while (!Firebase.ready() && (millis() - start) < 10000) {
        delay(100);
    }

    if (Firebase.ready()) {
        firebaseReady = true;
        Serial.println("[FIREBASE] Ready!");

        /* Set initial status */
        Firebase.RTDB.setString(&fbData, "petpal/status/mode", "IDLE");
        Firebase.RTDB.setBool(&fbData, "petpal/status/pet_present", false);
        Firebase.RTDB.setInt(&fbData, "petpal/status/uptime", 0);
    } else {
        Serial.println("[FIREBASE] Failed to initialize (will retry)");
    }
}


/* ========================= FIREBASE: PUSH SENSOR DATA ==================== */

void pushSensorData() {
    if (!firebaseReady) return;

    Firebase.RTDB.setFloat(&fbData, "petpal/sensors/temperature", lastTemp);
    Firebase.RTDB.setFloat(&fbData, "petpal/sensors/humidity", lastHumidity);
    Firebase.RTDB.setInt(&fbData, "petpal/sensors/water_level", lastWaterLevel);
    Firebase.RTDB.setInt(&fbData, "petpal/sensors/distance", lastDistance);
    Firebase.RTDB.setString(&fbData, "petpal/status/mode", stateToString(currentFSMState));
    Firebase.RTDB.setBool(&fbData, "petpal/status/pet_present", petPresent);
    Firebase.RTDB.setInt(&fbData, "petpal/status/uptime", millis() / 1000);

    /* Daily stats */
    Firebase.RTDB.setInt(&fbData, "petpal/stats/visits", dailyVisits);
    Firebase.RTDB.setInt(&fbData, "petpal/stats/feeds", dailyFeeds);
    Firebase.RTDB.setInt(&fbData, "petpal/stats/play_sessions", dailyPlaySessions);
    Firebase.RTDB.setInt(&fbData, "petpal/stats/interactions", dailyInteractions);
}


/* ========================= FIREBASE: POLL COMMANDS ======================= */

void pollFirebaseCommands() {
    if (!firebaseReady) return;

    /* Check for pending commands from the web dashboard */
    if (Firebase.RTDB.getString(&fbData, "petpal/commands/pending/action")) {
        String action = fbData.stringData();

        if (action.length() > 0 && action != "null") {
            Serial.println("[CMD] Received: " + action);

            if (action == "feed") {
                sendCommand(MSG_CMD_FEED);
            } else if (action == "play") {
                sendCommand(MSG_CMD_PLAY);
            } else if (action == "stop") {
                sendCommand(MSG_CMD_STOP);
            } else if (action == "call") {
                sendCommand(MSG_CMD_CALL_PET);
            } else if (action == "ai_report") {
                generateAIReport();
            }

            /* Clear the command after processing */
            Firebase.RTDB.setString(&fbData, "petpal/commands/pending/action", "");
            Firebase.RTDB.set(&fbData, "petpal/commands/pending/processed", true);
        }
    }

    /* Check for schedule updates */
    if (Firebase.RTDB.getJSON(&fbData, "petpal/schedule/feed_times")) {
        /* Parse and forward to MCXC444 if changed */
        /* Implementation depends on your schedule format */
    }
}


/* ========================= OPENAI API CALL =============================== */

void generateAIReport() {
    Serial.println("[AI] Generating daily report...");

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[AI] No WiFi, skipping report.");
        return;
    }

    /* Build the data summary for the AI */
    String dataSummary = "{";
    dataSummary += "\"date\":\"" + getDateString() + "\",";
    dataSummary += "\"visits\":" + String(dailyVisits) + ",";
    dataSummary += "\"feeds\":" + String(dailyFeeds) + ",";
    dataSummary += "\"play_sessions\":" + String(dailyPlaySessions) + ",";
    dataSummary += "\"interactions\":" + String(dailyInteractions) + ",";
    dataSummary += "\"water_alerts\":" + String(dailyWaterAlerts) + ",";
    dataSummary += "\"current_temp\":" + String(lastTemp, 1) + ",";
    dataSummary += "\"current_humidity\":" + String(lastHumidity, 1) + ",";
    dataSummary += "\"current_water_level\":" + String(lastWaterLevel) + ",";
    dataSummary += "\"event_log\":\"" + eventLog + "\"";
    dataSummary += "}";

    /* Build the OpenAI API request */
    StaticJsonDocument<2048> requestDoc;
    requestDoc["model"] = OPENAI_MODEL;
    requestDoc["max_tokens"] = 500;

    JsonArray messages = requestDoc.createNestedArray("messages");

    /* System prompt */
    JsonObject sysMsg = messages.createNestedObject();
    sysMsg["role"] = "system";
    sysMsg["content"] = "You are a veterinary-aware pet care assistant analyzing "
                        "daily activity data from an IoT pet monitoring station. "
                        "Provide: 1) A friendly 3-4 sentence summary of the pet's day. "
                        "2) Health observations (eating patterns, hydration, activity). "
                        "3) Any concerns with reasoning. "
                        "4) 1-2 actionable suggestions for tomorrow. "
                        "Keep the tone warm but informative.";

    /* User prompt with data */
    JsonObject userMsg = messages.createNestedObject();
    userMsg["role"] = "user";
    userMsg["content"] = "Here is today's pet activity data: " + dataSummary;

    String requestBody;
    serializeJson(requestDoc, requestBody);

    /* Make HTTP POST request */
    HTTPClient http;
    http.begin(OPENAI_ENDPOINT);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", String("Bearer ") + OPENAI_API_KEY);
    http.setTimeout(30000);  /* 30 second timeout for AI response */

    int httpCode = http.POST(requestBody);

    if (httpCode == 200) {
        String response = http.getString();

        /* Parse response */
        StaticJsonDocument<4096> responseDoc;
        DeserializationError err = deserializeJson(responseDoc, response);

        if (!err) {
            const char *content = responseDoc["choices"][0]["message"]["content"];
            if (content) {
                String aiSummary = String(content);
                Serial.println("[AI] Report generated:");
                Serial.println(aiSummary);

                /* Push to Firebase */
                if (firebaseReady) {
                    String reportPath = "petpal/ai_reports/" + getDateString();
                    Firebase.RTDB.setString(&fbData, reportPath + "/summary", aiSummary);
                    Firebase.RTDB.set(&fbData, reportPath + "/generated_at/.sv", "timestamp");
                    Firebase.RTDB.setInt(&fbData, reportPath + "/visits", dailyVisits);
                    Firebase.RTDB.setInt(&fbData, reportPath + "/feeds", dailyFeeds);
                }
            }
        } else {
            Serial.println("[AI] JSON parse error: " + String(err.c_str()));
        }
    } else {
        Serial.printf("[AI] HTTP error: %d\n", httpCode);
        if (httpCode > 0) {
            Serial.println(http.getString());
        }
    }

    http.end();
}


/* ========================= DHT11 READING ================================= */

void readDHT() {
    float temp = dht.readTemperature();
    float hum  = dht.readHumidity();

    if (!isnan(temp) && !isnan(hum)) {
        lastTemp     = temp;
        lastHumidity = hum;

        Serial.printf("[DHT] Temp: %.1f°C, Humidity: %.1f%%\n", temp, hum);

        /* Check temperature alerts */
        if (temp > TEMP_HIGH_ALERT || temp < TEMP_LOW_ALERT) {
            Serial.println("[DHT] Temperature alert!");
            /* The MCXC444 will handle the alarm; we just log it */
        }

        /* Send to MCXC444 */
        uint8_t payload[2];
        payload[0] = (uint8_t)temp;
        payload[1] = (uint8_t)hum;
        sendPacket(MSG_CMD_ESP_STATUS, payload, 2);
    } else {
        Serial.println("[DHT] Read failed (retrying next cycle).");
    }
}


/* ========================= UTILITY FUNCTIONS ============================= */

String getDateString() {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
        char buf[16];
        strftime(buf, sizeof(buf), "%Y-%m-%d", &timeinfo);
        return String(buf);
    }
    return "unknown";
}

String getTimeString() {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
        char buf[16];
        strftime(buf, sizeof(buf), "%H:%M:%S", &timeinfo);
        return String(buf);
    }
    return "unknown";
}

/* Reset daily counters (call at midnight) */
void resetDailyCounters() {
    dailyVisits = 0;
    dailyFeeds = 0;
    dailyPlaySessions = 0;
    dailyInteractions = 0;
    dailyWaterAlerts = 0;
    eventLog = "";
}

/* Check if it's midnight and reset counters */
void checkMidnightReset() {
    static int lastDay = -1;
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
        if (lastDay != -1 && timeinfo.tm_mday != lastDay) {
            /* New day — generate report for yesterday then reset */
            Serial.println("[SYSTEM] New day detected. Generating yesterday's report...");
            generateAIReport();
            resetDailyCounters();
        }
        lastDay = timeinfo.tm_mday;
    }
}


/* ========================= FEED SCHEDULE CHECK =========================== */
/*
 * Check if it's time to feed based on the stored schedule.
 * Sends MSG_CMD_FEED to MCXC444 at the scheduled times.
 */
void checkFeedSchedule() {
    static int lastFeedMinute = -1;

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) return;

    int currentMinute = timeinfo.tm_hour * 60 + timeinfo.tm_min;

    /* Avoid re-triggering in the same minute */
    if (currentMinute == lastFeedMinute) return;

    /* Check against schedule (default: 8:00 and 18:00) */
    /* These can be updated via Firebase commands */
    int scheduledTimes[] = { 8 * 60, 18 * 60 };  /* 8:00 AM, 6:00 PM */
    int numScheduled = sizeof(scheduledTimes) / sizeof(scheduledTimes[0]);

    for (int i = 0; i < numScheduled; i++) {
        if (currentMinute == scheduledTimes[i]) {
            Serial.printf("[SCHED] Feed time! (%02d:%02d)\n",
                          scheduledTimes[i] / 60, scheduledTimes[i] % 60);
            sendCommand(MSG_CMD_FEED);
            lastFeedMinute = currentMinute;
            return;
        }
    }
}


/* ========================= SETUP ========================================= */

void setup() {
    /* Debug serial */
    Serial.begin(115200);
    Serial.println();
    Serial.println("========================================");
    Serial.println("  PetPal ESP32 — Starting up...");
    Serial.println("========================================");

    /* UART to MCXC444 */
    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.println("[UART] Serial2 initialized (115200 baud)");

    /* DHT11 sensor */
    dht.begin();
    Serial.println("[DHT] Sensor initialized");

    /* WiFi */
    setupWiFi();

    /* NTP time sync */
    if (WiFi.status() == WL_CONNECTED) {
        setupTime();
    }

    /* Firebase */
    if (WiFi.status() == WL_CONNECTED) {
        setupFirebase();
    }

    Serial.println("[SYSTEM] Setup complete. Entering main loop.\n");
}


/* ========================= MAIN LOOP ===================================== */

void loop() {
    unsigned long now = millis();

    /* ---- 1. Read incoming UART data from MCXC444 ---- */
    while (Serial2.available()) {
        uint8_t byte = Serial2.read();
        if (parseByte(byte)) {
            processPacket();
        }
    }

    /* ---- 2. Read DHT11 sensor periodically ---- */
    if (now - lastDHTRead >= DHT_READ_INTERVAL) {
        lastDHTRead = now;
        readDHT();
    }

    /* ---- 3. Push sensor data to Firebase ---- */
    if (now - lastFirebasePush >= FIREBASE_PUSH_INTERVAL) {
        lastFirebasePush = now;
        pushSensorData();
    }

    /* ---- 4. Poll Firebase for commands from dashboard ---- */
    if (now - lastFirebasePoll >= FIREBASE_CMD_POLL) {
        lastFirebasePoll = now;
        pollFirebaseCommands();
    }

    /* ---- 5. Check feed schedule ---- */
    checkFeedSchedule();

    /* ---- 6. Check midnight reset ---- */
    checkMidnightReset();

    /* ---- 7. Auto AI report (if interval elapsed) ---- */
    if (now - lastAIReport >= AI_REPORT_INTERVAL) {
        lastAIReport = now;
        generateAIReport();
    }

    /* ---- 8. Reconnect WiFi if disconnected ---- */
    if (WiFi.status() != WL_CONNECTED) {
        static unsigned long lastReconnect = 0;
        if (now - lastReconnect > 30000) {
            lastReconnect = now;
            Serial.println("[WIFI] Reconnecting...");
            WiFi.disconnect();
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        }
    }

    /* Small delay to prevent CPU hogging */
    delay(10);
}
