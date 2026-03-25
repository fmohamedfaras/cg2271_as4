/*
 * =============================================================================
 * PetPal - UART Packet Protocol Implementation
 * =============================================================================
 * File:    protocol.c
 * =============================================================================
 */

#include "protocol.h"
#include <string.h>

/* Sequence number counter */
static uint8_t seqCounter = 0;


/* ========================= CRC-8/MAXIM ================================== */
/*
 * CRC-8/MAXIM (polynomial 0x31, init 0x00, no reflect, no xor)
 * Commonly used in embedded protocols for lightweight integrity checks.
 */
uint8_t protocol_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}


/* ========================= SEQUENCE NUMBER =============================== */

uint8_t protocol_next_seq(void)
{
    return seqCounter++;
}


/* ========================= BUILD PACKET ================================== */

uint8_t protocol_build_packet(uint8_t *buf, uint8_t type,
                              const uint8_t *payload, uint8_t payloadLen)
{
    if (payloadLen > PROTO_MAX_PAYLOAD) {
        payloadLen = PROTO_MAX_PAYLOAD;
    }

    uint8_t seq = protocol_next_seq();
    uint8_t length = 2 + payloadLen;  /* SEQ + TYPE + payload */

    /* Assemble packet */
    buf[0] = PROTO_START_BYTE;
    buf[1] = length;
    buf[2] = seq;
    buf[3] = type;

    if (payload != NULL && payloadLen > 0) {
        memcpy(&buf[4], payload, payloadLen);
    }

    /* CRC over SEQ + TYPE + PAYLOAD */
    uint8_t crc = protocol_crc8(&buf[2], length);
    buf[4 + payloadLen] = crc;

    return 4 + payloadLen + 1;  /* HEADER(4) + PAYLOAD + CRC(1) */
}


/* ========================= PARSER STATE MACHINE ========================== */

void protocol_parser_init(Parser_t *parser)
{
    memset(parser, 0, sizeof(Parser_t));
    parser->state = PARSE_WAIT_START;
}

bool protocol_parser_feed(Parser_t *parser, uint8_t byte)
{
    switch (parser->state) {

    case PARSE_WAIT_START:
        if (byte == PROTO_START_BYTE) {
            parser->packet.startByte = byte;
            parser->state = PARSE_WAIT_LENGTH;
        }
        break;

    case PARSE_WAIT_LENGTH:
        if (byte < 2 || byte > (2 + PROTO_MAX_PAYLOAD)) {
            /* Invalid length — reset */
            parser->state = PARSE_WAIT_START;
        } else {
            parser->packet.length = byte;
            parser->dataExpected = byte;   /* SEQ + TYPE + payload */
            parser->dataIdx = 0;
            parser->state = PARSE_WAIT_DATA;
        }
        break;

    case PARSE_WAIT_DATA:
        parser->buffer[parser->dataIdx++] = byte;

        if (parser->dataIdx >= parser->dataExpected) {
            /* Parse fields from buffer */
            parser->packet.seq  = parser->buffer[0];
            parser->packet.type = parser->buffer[1];
            parser->packet.payloadLen = parser->dataExpected - 2;

            if (parser->packet.payloadLen > 0) {
                memcpy(parser->packet.payload, &parser->buffer[2],
                       parser->packet.payloadLen);
            }

            parser->state = PARSE_WAIT_CRC;
        }
        break;

    case PARSE_WAIT_CRC:
        parser->packet.crc = byte;

        /* Verify CRC over the data portion (SEQ + TYPE + PAYLOAD) */
        {
            uint8_t computed = protocol_crc8(parser->buffer, parser->dataExpected);
            parser->state = PARSE_WAIT_START;

            if (computed == byte) {
                /* Valid packet received */
                return true;
            }
            /* CRC mismatch — silently discard, wait for next START */
        }
        break;

    default:
        parser->state = PARSE_WAIT_START;
        break;
    }

    return false;
}
