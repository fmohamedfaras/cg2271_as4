/*
 * =============================================================================
 * PetPal - UART Packet Protocol
 * =============================================================================
 * File:    protocol.h
 * Desc:    Packetized bi-directional UART protocol between MCXC444 and ESP32
 *
 * Packet format:
 *   [START_BYTE][LENGTH][SEQ][TYPE][PAYLOAD...][CRC8]
 *
 *   START_BYTE : 0xAA (fixed)
 *   LENGTH     : Total bytes from SEQ to end of PAYLOAD (excl START, LEN, CRC)
 *   SEQ        : Sequence number (0-255, wrapping)
 *   TYPE       : Message type identifier
 *   PAYLOAD    : 0-N bytes of data
 *   CRC8       : CRC-8/MAXIM over bytes from SEQ to end of PAYLOAD
 * =============================================================================
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* Protocol constants */
#define PROTO_START_BYTE        0xAA
#define PROTO_MAX_PAYLOAD       16
#define PROTO_HEADER_SIZE       4       /* START + LENGTH + SEQ + TYPE */
#define PROTO_OVERHEAD          5       /* HEADER + CRC */
#define PROTO_MAX_PACKET_SIZE   (PROTO_OVERHEAD + PROTO_MAX_PAYLOAD)

/* ------- Message types: MCXC444 → ESP32 (0x01 - 0x0F) ------- */
#define MSG_DISTANCE            0x01    /* Payload: uint16 distance_cm */
#define MSG_WATER_LEVEL         0x02    /* Payload: uint16 adc_value */
#define MSG_EVENT               0x03    /* Payload: uint8 event_code */
#define MSG_STATE               0x04    /* Payload: uint8 fsm_state */
#define MSG_STATUS              0x05    /* Payload: full status struct */
#define MSG_ACK                 0x06    /* Payload: uint8 acked_seq */

/* ------- Message types: ESP32 → MCXC444 (0x10 - 0x1F) ------- */
#define MSG_CMD_FEED            0x10    /* Payload: none (trigger feed now) */
#define MSG_CMD_PLAY            0x11    /* Payload: uint8 pattern_id */
#define MSG_CMD_STOP            0x12    /* Payload: none (stop all) */
#define MSG_CMD_CALL_PET        0x13    /* Payload: none (buzzer call) */
#define MSG_CMD_SET_SCHEDULE    0x14    /* Payload: uint8 slot, uint8 hour, uint8 min, uint8 en */
#define MSG_CMD_SET_THRESHOLD   0x15    /* Payload: uint8 param_id, uint16 value */
#define MSG_CMD_ESP_STATUS      0x16    /* Payload: uint8 temp, uint8 humidity */
#define MSG_CMD_ACK             0x17    /* Payload: uint8 acked_seq */


/* ========================= PACKET STRUCTURE ============================== */

typedef struct {
    uint8_t  startByte;
    uint8_t  length;                        /* SEQ + TYPE + payload length */
    uint8_t  seq;
    uint8_t  type;
    uint8_t  payload[PROTO_MAX_PAYLOAD];
    uint8_t  payloadLen;                    /* Actual payload bytes (derived) */
    uint8_t  crc;
} Packet_t;


/* ========================= PARSER STATE MACHINE ========================== */

typedef enum {
    PARSE_WAIT_START,
    PARSE_WAIT_LENGTH,
    PARSE_WAIT_DATA,
    PARSE_WAIT_CRC
} ParseState_t;

typedef struct {
    ParseState_t state;
    Packet_t     packet;
    uint8_t      dataIdx;
    uint8_t      dataExpected;
    uint8_t      buffer[PROTO_MAX_PACKET_SIZE];
} Parser_t;


/* ========================= API FUNCTIONS ================================= */

/*
 * Initialize the packet parser state machine.
 */
void protocol_parser_init(Parser_t *parser);

/*
 * Feed one byte into the parser. Returns true if a complete valid packet
 * has been received (accessible via parser->packet).
 */
bool protocol_parser_feed(Parser_t *parser, uint8_t byte);

/*
 * Build a packet into the provided buffer. Returns total packet size.
 *
 * @param buf       Output buffer (must be >= PROTO_MAX_PACKET_SIZE)
 * @param type      Message type
 * @param payload   Payload data (can be NULL if payloadLen == 0)
 * @param payloadLen Number of payload bytes
 * @return          Total packet size in bytes
 */
uint8_t protocol_build_packet(uint8_t *buf, uint8_t type,
                              const uint8_t *payload, uint8_t payloadLen);

/*
 * Compute CRC-8/MAXIM over given data.
 */
uint8_t protocol_crc8(const uint8_t *data, uint8_t len);

/*
 * Get the next sequence number (auto-incrementing, wrapping).
 */
uint8_t protocol_next_seq(void);


#endif /* PROTOCOL_H */
