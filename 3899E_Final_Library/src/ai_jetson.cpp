/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:     ai_jetson.cpp                                               */
/*    Based on:   VEX Robotics VAIC_25_26 ai_jetson.cpp by James Pearman     */
/*    Modified:   Team 3899 VAIRC Push Back                                   */
/*                                                                            */
/*    Key changes from original:                                              */
/*      1. Serial I/O: getchar()/fopen replaced by vex::serial on Smart Port */
/*      2. Constructor takes port number parameter                            */
/*      3. request_map() removed entirely — Jetson pushes asynchronously     */
/*      4. kStateGoodPacket: third memcpy added for strategyCode/reserved     */
/*      5. get_strategy() convenience getter added                            */
/*      6. Baud rate: 460800 (was 115200)                                     */
/*      7. Parser state machine and CRC32 unchanged from original             */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "ai_jetson.h"

using namespace vex;
using namespace ai;

// ── CRC32 table (static, shared across all instances) ────────────────────────
uint32_t jetson::_crc32_table[256];

/*----------------------------------------------------------------------------*/
/** @brief  Constructor — opens Smart Port serial at 460800 baud             */
/** @param  port  V5 Smart Port number (1-21)                                 */
/*----------------------------------------------------------------------------*/
jetson::jetson(int32_t port) : _port(port) {
    state = jetson_state::kStateSyncWait1;

    // High-priority receive task — same pattern as original
    thread t1 = thread(receive_task, static_cast<void *>(this));
    t1.setPriority(thread::threadPriorityHigh);
}

jetson::~jetson() {
}

/*----------------------------------------------------------------------------*/
// Diagnostic getters — unchanged from original
/*----------------------------------------------------------------------------*/
int32_t jetson::get_packets()  { return packets; }
int32_t jetson::get_errors()   { return errors; }
int32_t jetson::get_timeouts() { return timeouts; }
int32_t jetson::get_total()    { return total_data_received; }

/*----------------------------------------------------------------------------*/
/** @brief  Copy latest received AI_RECORD to caller's buffer                */
/** @return Payload length of last good packet, or 0 if none received yet    */
/*----------------------------------------------------------------------------*/
int32_t
jetson::get_data(AI_RECORD *map) {
    int32_t length = 0;
    if (map != NULL) {
        maplock.lock();
        memcpy(map, &last_map, sizeof(AI_RECORD));
        length = last_payload_length;
        maplock.unlock();
    }
    return length;
}

/*----------------------------------------------------------------------------*/
/** @brief  Convenience getter for strategy code from latest packet          */
/*----------------------------------------------------------------------------*/
int32_t
jetson::get_strategy(void) {
    maplock.lock();
    int32_t code = last_map.strategyCode;
    maplock.unlock();
    return code;
}

/*----------------------------------------------------------------------------*/
/** @brief  CRC32 — identical algorithm to original and vaic_protocol.py     */
/*----------------------------------------------------------------------------*/
uint32_t
jetson::crc32(uint8_t *pData, uint32_t numberOfBytes, uint32_t accumulator) {
    uint32_t i, j;
    const uint32_t POLYNOMIAL_CRC32 = 0x04C11DB7;

    // Build table on first call
    if (_crc32_table[1] == 0) {
        uint32_t crc_accum;
        for (i = 0; i < 256; i++) {
            crc_accum = i << 24;
            for (j = 0; j < 8; j++) {
                if (crc_accum & 0x80000000L)
                    crc_accum = (crc_accum << 1) ^ POLYNOMIAL_CRC32;
                else
                    crc_accum = (crc_accum << 1);
            }
            _crc32_table[i] = crc_accum;
        }
    }

    for (j = 0; j < numberOfBytes; j++) {
        i = ((accumulator >> 24) ^ *pData++) & 0xFF;
        accumulator = (accumulator << 8) ^ _crc32_table[i];
    }
    return accumulator;
}

/*----------------------------------------------------------------------------*/
/** @brief  Parse one received byte — state machine unchanged from original  */
/** @return true if more processing needed (recall immediately)              */
/*----------------------------------------------------------------------------*/
bool
jetson::parse(uint8_t data) {
    bool bRecall = false;

    // 250ms inter-byte timeout — resets state machine on stalled packet
    // At 460800 baud a 300-byte packet transmits in ~5ms, so 250ms is generous
    if (state != jetson_state::kStateSyncWait1 && timer.time() > 250) {
        timeouts++;
        state = jetson_state::kStateSyncWait1;
    }
    timer.clear();

    switch (state) {
      // ── Sync sequence ─────────────────────────────────────────────────────
      case jetson_state::kStateSyncWait1:
        if (static_cast<sync_byte>(data) == sync_byte::kSync1)
            state = jetson_state::kStateSyncWait2;
        break;

      case jetson_state::kStateSyncWait2:
        state = jetson_state::kStateSyncWait1;
        if (static_cast<sync_byte>(data) == sync_byte::kSync2)
            state = jetson_state::kStateSyncWait3;
        break;

      case jetson_state::kStateSyncWait3:
        state = jetson_state::kStateSyncWait1;
        if (static_cast<sync_byte>(data) == sync_byte::kSync3)
            state = jetson_state::kStateSyncWait4;
        break;

      case jetson_state::kStateSyncWait4:
        state = jetson_state::kStateSyncWait1;
        if (static_cast<sync_byte>(data) == sync_byte::kSync4) {
            state = jetson_state::kStateLength;
            index = 0;
            payload_length = 0;
        }
        break;

      // ── Payload length (2 bytes little-endian) ────────────────────────────
      case jetson_state::kStateLength:
        payload_length = (payload_length >> 8) + ((uint16_t)data << 8);
        if (index++ == 1) {
            state = jetson_state::kStateSpare;
            index = 0;
            payload_type = 0;
        }
        break;

      // ── Packet type (2 bytes little-endian) ──────────────────────────────
      case jetson_state::kStateSpare:
        payload_type = (payload_type >> 8) + ((uint16_t)data << 8);
        if (index++ == 1) {
            state = jetson_state::kStateCrc32;
            index = 0;
            payload_crc32 = 0;
        }
        break;

      // ── CRC32 (4 bytes little-endian) ─────────────────────────────────────
      case jetson_state::kStateCrc32:
        payload_crc32 = (payload_crc32 >> 8) + ((uint32_t)data << 24);
        if (index++ == 3) {
            state = jetson_state::kStatePayload;
            index = 0;
            calc_crc32 = 0;
        }
        break;

      // ── Payload data ──────────────────────────────────────────────────────
      case jetson_state::kStatePayload:
        if (index < sizeof(payload)) {
            payload.bytes[index] = data;
            index++;
            // Running CRC — avoids recalculating over whole buffer at end
            calc_crc32 = crc32(&data, 1, calc_crc32);

            if (index == payload_length) {
                if (payload_crc32 == calc_crc32)
                    state = jetson_state::kStateGoodPacket;
                else
                    state = jetson_state::kStateBadPacket;
                bRecall = true;
            }
        } else {
            state = jetson_state::kStateBadPacket;
            bRecall = true;
        }
        break;

      // ── Good packet — deserialize AI_RECORD ──────────────────────────────
      case jetson_state::kStateGoodPacket:
        if (payload_type == MAP_PACKET_TYPE) {
            AI_RECORD newMap;
            memset(&newMap, 0, sizeof(newMap));

            // ── memcpy 1: detectionCount + POS_RECORD (44 bytes) ─────────
            memcpy(&newMap, &payload.bytes[0], MAP_POS_SIZE);

            // Clamp detection count — never trust unvalidated input
            if (newMap.detectionCount > MAX_DETECTIONS)
                newMap.detectionCount = MAX_DETECTIONS;

            // ── memcpy 2: DETECTION_OBJECT array ─────────────────────────
            uint32_t det_bytes = sizeof(DETECTION_OBJECT) * newMap.detectionCount;
            memcpy(&newMap.detections,
                   &payload.bytes[MAP_POS_SIZE],
                   det_bytes);

            // ── memcpy 3: Extension fields (strategyCode + reserved) ──────
            // Guard: only read if payload includes extension bytes.
            // This makes the parser backward-compatible if extension is absent.
            uint32_t ext_offset = MAP_POS_SIZE + det_bytes;
            uint32_t ext_size   = sizeof(int32_t)       // strategyCode
                                + sizeof(int32_t) * 3;  // reserved[3]

            if (payload_length >= ext_offset + ext_size) {
                memcpy(&newMap.strategyCode,
                       &payload.bytes[ext_offset],
                       sizeof(int32_t));
                memcpy(&newMap.reserved,
                       &payload.bytes[ext_offset + sizeof(int32_t)],
                       sizeof(int32_t) * 3);
            }

            // ── Thread-safe update ────────────────────────────────────────
            maplock.lock();
            memcpy(&last_map, &newMap, sizeof(AI_RECORD));
            last_payload_length = payload_length;
            maplock.unlock();
        }

        last_packet_time = timer.system();
        packets++;
        state = jetson_state::kStateSyncWait1;
        break;

      // ── Bad packet ────────────────────────────────────────────────────────
      case jetson_state::kStateBadPacket:
        errors++;
        state = jetson_state::kStateSyncWait1;
        break;

      default:
        state = jetson_state::kStateSyncWait1;
        break;
    }

    return bRecall;
}

/*----------------------------------------------------------------------------*/
/** @brief  Background receive task — reads Smart Port, feeds parser         */
/*                                                                            */
/** Key difference from original:                                             */
/*   Original used getchar() on USB CDC stdin (/dev/serial1).                */
/*   This version uses vex::serial on a Smart Port at 460800 baud.           */
/*   vex::serial::read() is non-blocking; we sleep briefly when no data      */
/*   to avoid spinning the CPU, but sleep is short enough not to add latency.*/
/*----------------------------------------------------------------------------*/
int
jetson::receive_task(void *arg) {
    if (arg == NULL) return 0;

    jetson *instance = static_cast<jetson *>(arg);

    // Open Smart Port serial at 460800 baud
    // Port number set in constructor, matches physical RS485 wiring
    vexGenericSerialEnable(instance->_port - 1, 0);
    vexGenericSerialBaudrate(instance->_port - 1, JETSON_BAUD_RATE);

    while (1) {
        uint8_t buf[1];
        int rxchar = vexGenericSerialReceive(instance->_port - 1, buf, 1) > 0 ? buf[0] : -1;

        if (rxchar >= 0) {
            instance->total_data_received++;
            // parse() returns true if state machine needs another cycle
            // (kStateGoodPacket / kStateBadPacket need one more call)
            while (instance->parse((uint8_t)rxchar))
                this_thread::yield();
        } else {
            // No byte available — yield to other tasks rather than spin
            // 1ms sleep adds negligible latency (packet arrives every ~7ms)
            this_thread::sleep_for(1);
        }
    }
    return 0;
}
