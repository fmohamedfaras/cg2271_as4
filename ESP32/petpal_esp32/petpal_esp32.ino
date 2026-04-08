/*
 * =============================================================================
 * PetPal - ESP32 Sensor + Firebase + UART Bridge
 * =============================================================================
 * ESP32 responsibilities:
 *   - Read DHT11, water level, and shock sensor
 *   - Control laser and buzzer
 *   - Post sensor/status data to Firebase using HTTP JSON
 *   - Poll Firebase for commands and forward MCXC444 actions over UART
 *
 * MCXC444 responsibilities:
 *   - Servo control
 *   - Ultrasonic handling
 *
 * Required library:
 *   - DHT sensor library by Adafruit
 * =============================================================================
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <DHT.h>
#include <string.h>
#include "../../MCXC444/source/protocol.h"

/* ========================= PIN CONSTANTS ================================= */
static const int SHOCK_PIN = 5;
static const int WATER_PIN = 3;
static const int DHT_PIN = 11;
static const int BUZZER_PIN = 7;
static const int LASER_PIN = 9;
static const int UART_RX_PIN = 44;
static const int UART_TX_PIN = 43;

/* ========================= WIFI / FIREBASE =============================== */
static const char *WIFI_SSID = "YOUR_WIFI_SSID";
static const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

/*
 * Use full Firebase REST URLs here.
 * Examples:
 *   https://your-project-default-rtdb.asia-southeast1.firebasedatabase.app/sensors.json?auth=TOKEN
 *   https://your-project-default-rtdb.asia-southeast1.firebasedatabase.app/commands.json?auth=TOKEN
 */
static const char *FIREBASE_POST_URL = "https://your-project.firebaseio.com/sensors.json";
static const char *FIREBASE_COMMANDS_URL = "https://your-project.firebaseio.com/commands.json";

/* ========================= APP CONFIG ==================================== */
static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t UART_BAUD = 115200;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
static const uint32_t WIFI_RETRY_INTERVAL_MS = 10000;
static const uint32_t WATER_READ_INTERVAL_MS = 1000;
static const uint32_t DHT_READ_INTERVAL_MS = 3000;
static const uint32_t FIREBASE_POST_INTERVAL_MS = 10000;
static const uint32_t FIREBASE_COMMAND_INTERVAL_MS = 3000;
static const uint32_t STATUS_PRINT_INTERVAL_MS = 5000;
static const uint32_t SHOCK_DEBOUNCE_MS = 200;
static const uint16_t WATER_LOW_THRESHOLD = 500;
static const uint16_t BUZZER_DEFAULT_FREQ = 2000;
static const uint8_t PLAY_PATTERN_ID = 1;
static const uint8_t BUZZER_LEDC_CHANNEL = 0;
static const uint8_t BUZZER_LEDC_RESOLUTION = 8;

/* ========================= DHT =========================================== */
static const uint8_t DHT_TYPE = DHT11;
DHT dht(DHT_PIN, DHT_TYPE);

/* ========================= STATE ========================================= */
Parser_t uartParser;

volatile bool shockInterruptFlag = false;
volatile uint32_t shockCount = 0;
volatile unsigned long lastShockInterruptMs = 0;

float lastTemperature = 0.0f;
float lastHumidity = 0.0f;
uint16_t lastWaterLevel = 0;
bool shockEventLatched = false;
bool laserStatus = false;
bool buzzerStatus = false;

uint16_t lastDistanceCm = 0;
bool previousDispenseTreat = false;
bool previousPlayMode = false;

bool buzzerTimedMode = false;
uint32_t buzzerOffAtMs = 0;
uint16_t buzzerFrequency = 0;

unsigned long lastWaterReadMs = 0;
unsigned long lastDhtReadMs = 0;
unsigned long lastFirebasePostMs = 0;
unsigned long lastCommandCheckMs = 0;
unsigned long lastStatusPrintMs = 0;
unsigned long lastWiFiAttemptMs = 0;

/* ========================= HELPERS ======================================= */
uint8_t protocol_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x31;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

uint8_t protocol_next_seq(void)
{
    static uint8_t seqCounter = 0;
    return seqCounter++;
}

uint8_t protocol_build_packet(uint8_t *buf, uint8_t type, const uint8_t *payload, uint8_t payloadLen)
{
    if (payloadLen > PROTO_MAX_PAYLOAD)
    {
        payloadLen = PROTO_MAX_PAYLOAD;
    }

    const uint8_t seq = protocol_next_seq();
    const uint8_t length = 2 + payloadLen;

    buf[0] = PROTO_START_BYTE;
    buf[1] = length;
    buf[2] = seq;
    buf[3] = type;

    for (uint8_t i = 0; i < payloadLen; i++)
    {
        buf[4 + i] = payload[i];
    }

    buf[4 + payloadLen] = protocol_crc8(&buf[2], length);
    return 5 + payloadLen;
}

void protocol_parser_init(Parser_t *parser)
{
    memset(parser, 0, sizeof(Parser_t));
    parser->state = PARSE_WAIT_START;
}

bool protocol_parser_feed(Parser_t *parser, uint8_t byte)
{
    switch (parser->state)
    {
    case PARSE_WAIT_START:
        if (byte == PROTO_START_BYTE)
        {
            parser->packet.startByte = byte;
            parser->state = PARSE_WAIT_LENGTH;
        }
        break;

    case PARSE_WAIT_LENGTH:
        if (byte < 2 || byte > (2 + PROTO_MAX_PAYLOAD))
        {
            parser->state = PARSE_WAIT_START;
        }
        else
        {
            parser->packet.length = byte;
            parser->dataExpected = byte;
            parser->dataIdx = 0;
            parser->state = PARSE_WAIT_DATA;
        }
        break;

    case PARSE_WAIT_DATA:
        parser->buffer[parser->dataIdx++] = byte;
        if (parser->dataIdx >= parser->dataExpected)
        {
            parser->packet.seq = parser->buffer[0];
            parser->packet.type = parser->buffer[1];
            parser->packet.payloadLen = parser->dataExpected - 2;

            for (uint8_t i = 0; i < parser->packet.payloadLen; i++)
            {
                parser->packet.payload[i] = parser->buffer[2 + i];
            }

            parser->state = PARSE_WAIT_CRC;
        }
        break;

    case PARSE_WAIT_CRC:
    {
        parser->packet.crc = byte;
        const uint8_t computed = protocol_crc8(parser->buffer, parser->dataExpected);
        parser->state = PARSE_WAIT_START;
        return computed == byte;
    }

    default:
        parser->state = PARSE_WAIT_START;
        break;
    }

    return false;
}

bool beginHttpClient(HTTPClient &http, WiFiClientSecure &client, const char *url)
{
    client.setInsecure();
    return http.begin(client, url);
}

bool extractJsonBool(const String &json, const char *key, bool defaultValue)
{
    const String needle = String("\"") + key + "\":";
    int index = json.indexOf(needle);
    if (index < 0)
    {
        return defaultValue;
    }

    index += needle.length();
    while (index < json.length() && (json[index] == ' ' || json[index] == '\n' || json[index] == '\r' || json[index] == '\t'))
    {
        index++;
    }

    if (json.startsWith("true", index))
    {
        return true;
    }
    if (json.startsWith("false", index))
    {
        return false;
    }
    return defaultValue;
}

bool patchFirebaseCommand(const char *key, bool value)
{
    if (WiFi.status() != WL_CONNECTED)
    {
        return false;
    }

    WiFiClientSecure client;
    HTTPClient http;

    if (!beginHttpClient(http, client, FIREBASE_COMMANDS_URL))
    {
        Serial.println("[FIREBASE] Failed to open commands URL for PATCH");
        return false;
    }

    http.addHeader("Content-Type", "application/json");
    const String body = String("{\"") + key + "\":" + (value ? "true" : "false") + "}";
    const int httpCode = http.sendRequest("PATCH", body);

    if (httpCode > 0)
    {
        Serial.printf("[FIREBASE] PATCH %s -> %s (HTTP %d)\n", key, value ? "true" : "false", httpCode);
    }
    else
    {
        Serial.printf("[FIREBASE] PATCH failed for %s: %s\n", key, http.errorToString(httpCode).c_str());
    }

    http.end();
    return httpCode > 0 && httpCode < 300;
}

/* ========================= ISR =========================================== */
void IRAM_ATTR shockISR()
{
    const unsigned long now = millis();
    if ((now - lastShockInterruptMs) >= SHOCK_DEBOUNCE_MS)
    {
        lastShockInterruptMs = now;
        shockCount++;
        shockInterruptFlag = true;
    }
}

/* ========================= REQUIRED FUNCTIONS ============================ */
void controlLaser(bool enabled)
{
    laserStatus = enabled;
    digitalWrite(LASER_PIN, enabled ? HIGH : LOW);
}

void controlBuzzer(bool enabled, uint16_t frequency = BUZZER_DEFAULT_FREQ, uint32_t durationMs = 0)
{
    buzzerStatus = enabled;
    buzzerFrequency = enabled ? frequency : 0;
    buzzerTimedMode = enabled && durationMs > 0;
    buzzerOffAtMs = buzzerTimedMode ? (millis() + durationMs) : 0;

    if (enabled)
    {
        ledcWriteTone(BUZZER_LEDC_CHANNEL, buzzerFrequency);
    }
    else
    {
        ledcWriteTone(BUZZER_LEDC_CHANNEL, 0);
    }
}

void sendUARTCommand(uint8_t msgType, const uint8_t *payload = nullptr, uint8_t payloadLen = 0)
{
    uint8_t packet[PROTO_MAX_PACKET_SIZE] = {0};
    const uint8_t packetLen = protocol_build_packet(packet, msgType, payload, payloadLen);
    Serial1.write(packet, packetLen);
    Serial.printf("[UART] Sent type 0x%02X, payloadLen=%u\n", msgType, payloadLen);
}

void setupWiFi()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        return;
    }

    Serial.printf("[WIFI] Connecting to %s\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWiFiAttemptMs = millis();

    while (WiFi.status() != WL_CONNECTED && (millis() - lastWiFiAttemptMs) < WIFI_CONNECT_TIMEOUT_MS)
    {
        if (buzzerTimedMode && millis() >= buzzerOffAtMs)
        {
            controlBuzzer(false);
        }
        yield();
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.printf("[WIFI] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
    }
    else
    {
        Serial.println("[WIFI] Connection timeout. Loop will retry.");
    }
}

void readSensors()
{
    const unsigned long now = millis();

    if (shockInterruptFlag)
    {
        noInterrupts();
        shockInterruptFlag = false;
        interrupts();

        shockEventLatched = true;
        controlBuzzer(true, 2400, 120);
        Serial.printf("[SHOCK] Event detected. Count=%lu\n", shockCount);
    }

    if (now - lastWaterReadMs >= WATER_READ_INTERVAL_MS)
    {
        lastWaterReadMs = now;
        lastWaterLevel = analogRead(WATER_PIN);
    }

    if (now - lastDhtReadMs >= DHT_READ_INTERVAL_MS)
    {
        lastDhtReadMs = now;
        const float temperature = dht.readTemperature();
        const float humidity = dht.readHumidity();

        if (!isnan(temperature) && !isnan(humidity))
        {
            lastTemperature = temperature;
            lastHumidity = humidity;
        }
        else
        {
            Serial.println("[DHT] Read failed");
        }
    }

    if (buzzerTimedMode && now >= buzzerOffAtMs)
    {
        controlBuzzer(false);
    }
}

void sendToFirebase()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        return;
    }

    WiFiClientSecure client;
    HTTPClient http;

    if (!beginHttpClient(http, client, FIREBASE_POST_URL))
    {
        Serial.println("[FIREBASE] Failed to open sensor URL");
        return;
    }

    http.addHeader("Content-Type", "application/json");

    const bool shockToSend = shockEventLatched;
    String json = "{";
    json += "\"temperature\":" + String(lastTemperature, 1) + ",";
    json += "\"humidity\":" + String(lastHumidity, 1) + ",";
    json += "\"waterLevel\":" + String(lastWaterLevel) + ",";
    json += "\"shockEvent\":" + String(shockToSend ? "true" : "false") + ",";
    json += "\"laserStatus\":" + String(laserStatus ? "true" : "false") + ",";
    json += "\"buzzerStatus\":" + String(buzzerStatus ? "true" : "false");
    json += "}";

    const int httpCode = http.POST(json);
    if (httpCode > 0)
    {
        Serial.printf("[FIREBASE] POST success (HTTP %d)\n", httpCode);
        if (httpCode < 300)
        {
            shockEventLatched = false;
        }
    }
    else
    {
        Serial.printf("[FIREBASE] POST failed: %s\n", http.errorToString(httpCode).c_str());
    }

    http.end();
}

void checkCommands()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        return;
    }

    WiFiClientSecure client;
    HTTPClient http;

    if (!beginHttpClient(http, client, FIREBASE_COMMANDS_URL))
    {
        Serial.println("[FIREBASE] Failed to open commands URL");
        return;
    }

    const int httpCode = http.GET();
    if (httpCode <= 0)
    {
        Serial.printf("[FIREBASE] GET commands failed: %s\n", http.errorToString(httpCode).c_str());
        http.end();
        return;
    }

    if (httpCode >= 300)
    {
        Serial.printf("[FIREBASE] GET commands returned HTTP %d\n", httpCode);
        http.end();
        return;
    }

    const String response = http.getString();
    http.end();

    const bool dispenseTreat = extractJsonBool(response, "dispenseTreat", false);
    const bool playMode = extractJsonBool(response, "playMode", false);

    if (dispenseTreat && !previousDispenseTreat)
    {
        sendUARTCommand(MSG_CMD_FEED);
        patchFirebaseCommand("dispenseTreat", false);
    }

    if (playMode && !previousPlayMode)
    {
        const uint8_t payload[1] = {PLAY_PATTERN_ID};
        controlLaser(true);
        sendUARTCommand(MSG_CMD_PLAY, payload, sizeof(payload));
    }
    else if (!playMode && previousPlayMode)
    {
        controlLaser(false);
        sendUARTCommand(MSG_CMD_STOP);
    }

    previousDispenseTreat = dispenseTreat;
    previousPlayMode = playMode;
}

/* ========================= UART RX HANDLING ============================== */
void handleIncomingPacket(const Packet_t &packet)
{
    if (packet.type == MSG_DISTANCE && packet.payloadLen >= 2)
    {
        lastDistanceCm = (static_cast<uint16_t>(packet.payload[0]) << 8) | packet.payload[1];
        Serial.printf("[UART] Distance update from MCXC444: %u cm\n", lastDistanceCm);
    }
    else if (packet.type == MSG_ACK && packet.payloadLen >= 1)
    {
        Serial.printf("[UART] ACK received for seq %u\n", packet.payload[0]);
    }
    else if (packet.type == MSG_EVENT && packet.payloadLen >= 1)
    {
        Serial.printf("[UART] Event code from MCXC444: %u\n", packet.payload[0]);
    }
    else
    {
        Serial.printf("[UART] Received packet type 0x%02X, payloadLen=%u\n", packet.type, packet.payloadLen);
    }
}

void processUART()
{
    while (Serial1.available())
    {
        const uint8_t incoming = static_cast<uint8_t>(Serial1.read());
        if (protocol_parser_feed(&uartParser, incoming))
        {
            handleIncomingPacket(uartParser.packet);
        }
    }
}

/* ========================= ARDUINO SETUP/LOOP =========================== */
void setup()
{
    Serial.begin(SERIAL_BAUD);
    Serial.println();
    Serial.println("========================================");
    Serial.println("  PetPal ESP32 Bridge");
    Serial.println("  Sensors + Firebase + UART");
    Serial.println("========================================");

    pinMode(SHOCK_PIN, INPUT_PULLUP);
    pinMode(WATER_PIN, INPUT);
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW);

    analogReadResolution(12);
    attachInterrupt(digitalPinToInterrupt(SHOCK_PIN), shockISR, FALLING);

    ledcSetup(BUZZER_LEDC_CHANNEL, 1000, BUZZER_LEDC_RESOLUTION);
    ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);
    ledcWriteTone(BUZZER_LEDC_CHANNEL, 0);

    dht.begin();
    Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    protocol_parser_init(&uartParser);

    setupWiFi();

    Serial.printf("[INIT] Shock pin: %d\n", SHOCK_PIN);
    Serial.printf("[INIT] Water pin: %d\n", WATER_PIN);
    Serial.printf("[INIT] DHT pin: %d\n", DHT_PIN);
    Serial.printf("[INIT] Buzzer pin: %d\n", BUZZER_PIN);
    Serial.printf("[INIT] Laser pin: %d\n", LASER_PIN);
    Serial.printf("[INIT] UART TX=%d RX=%d\n", UART_TX_PIN, UART_RX_PIN);
}

void loop()
{
    const unsigned long now = millis();

    if (WiFi.status() != WL_CONNECTED && (now - lastWiFiAttemptMs) >= WIFI_RETRY_INTERVAL_MS)
    {
        setupWiFi();
    }

    processUART();
    readSensors();

    if (now - lastCommandCheckMs >= FIREBASE_COMMAND_INTERVAL_MS)
    {
        lastCommandCheckMs = now;
        checkCommands();
    }

    if (now - lastFirebasePostMs >= FIREBASE_POST_INTERVAL_MS)
    {
        lastFirebasePostMs = now;
        sendToFirebase();
    }

    if (now - lastStatusPrintMs >= STATUS_PRINT_INTERVAL_MS)
    {
        lastStatusPrintMs = now;
        Serial.println("--- STATUS ---");
        Serial.printf("Temp: %.1f C\n", lastTemperature);
        Serial.printf("Humidity: %.1f %%\n", lastHumidity);
        Serial.printf("Water: %u%s\n", lastWaterLevel, lastWaterLevel < WATER_LOW_THRESHOLD ? " LOW" : "");
        Serial.printf("Shock count: %lu\n", shockCount);
        Serial.printf("Shock latched: %s\n", shockEventLatched ? "YES" : "NO");
        Serial.printf("Laser: %s\n", laserStatus ? "ON" : "OFF");
        Serial.printf("Buzzer: %s\n", buzzerStatus ? "ON" : "OFF");
        Serial.printf("MCXC distance: %u cm\n", lastDistanceCm);
        Serial.printf("WiFi: %s\n", WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
        Serial.println("--------------");
    }
}
