/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:     ai_jetson.cpp                                               */
/*    Based on:   VEX Robotics VAIC_25_26 ai_jetson.cpp by James Pearman     */
/*    Modified:   Team 3899 VAIRC Push Back                                   */
/*                                                                            */
/*    Key changes from original VEXcode version:                              */
/*      1. Serial I/O: vexGenericSerial* replaced by pros::Serial             */
/*      2. vex::thread replaced by pros::Task at TASK_PRIORITY_DEFAULT+2      */
/*      3. timer.time() / timer.clear() → timer.value() / timer.reset()       */
/*         (matching the PROS Timer wrapper defined in ai_jetson.h)           */
/*      4. timer.system() → pros::millis()                                    */
/*      5. this_thread::yield() / sleep_for() → pros::delay()                */
/*      6. receive_task return type: int → void (PROS task signature)         */
/*      7. Parser state machine, CRC32, and deserialization unchanged         */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "ai_jetson.h"
#include <cstring>

using namespace ai;

// ── CRC32 lookup table (static, shared across all instances) ─────────────────
uint32_t jetson::_crc32_table[256];

/*----------------------------------------------------------------------------*/
/** @brief  Constructor — opens Smart Port serial at JETSON_BAUD_RATE baud   */
/** @param  port  V5 Smart Port number (1-indexed, matches physical wiring)   */
/*----------------------------------------------------------------------------*/
jetson::jetson(int32_t port) : _port(port) {
    state = jetson_state::kStateSyncWait1;

    // Launch high-priority background receive task.
    // TASK_PRIORITY_DEFAULT+2 matches the original threadPriorityHigh intent.
    pros::Task(receive_task, static_cast<void*>(this),
               TASK_PRIORITY_DEFAULT + 2, TASK_STACK_DEPTH_DEFAULT, "JetsonRx");
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
/** @brief  Thread-safe copy of the latest AI_RECORD into the caller's buffer */
/** @return Payload length of last good packet, or 0 if none received yet     */
/*----------------------------------------------------------------------------*/
int32_t
jetson::get_data(AI_RECORD* map) {
    int32_t length = 0;
    if (map != nullptr) {
        maplock.lock();
        memcpy(map, &last_map, sizeof(AI_RECORD));
        length = last_payload_length;
        maplock.unlock();
    }
    return length;
}

/*----------------------------------------------------------------------------*/
/** @brief  Convenience getter — returns only strategyCode from latest packet */
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
jetson::crc32(uint8_t* pData, uint32_t numberOfBytes, uint32_t accumulator) {
    uint32_t i, j;
    const uint32_t POLYNOMIAL_CRC32 = 0x04C11DB7;

    // Build the lookup table on the first call (lazy initialization)
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

    // Accumulate CRC one byte at a time
    for (j = 0; j < numberOfBytes; j++) {
        i = ((accumulator >> 24) ^ *pData++) & 0xFF;
        accumulator = (accumulator << 8) ^ _crc32_table[i];
    }
    return accumulator;
}

/*----------------------------------------------------------------------------*/
/** @brief  Parse one received byte through the packet state machine          */
/** @return true if the state machine needs another immediate pass            */
/*                                                                            */
/** State machine is unchanged from the original James Pearman implementation.*/
/** Timer calls updated to match the PROS Timer wrapper in ai_jetson.h:       */
/**   timer.time()  → timer.value()   (elapsed ms since last reset)          */
/**   timer.clear() → timer.reset()   (restart the elapsed counter)          */
/*----------------------------------------------------------------------------*/
bool
jetson::parse(uint8_t data) {
    bool bRecall = false;

    // 250 ms inter-byte timeout — resets the state machine on a stalled packet.
    // At 460800 baud a 300-byte packet transmits in ~5 ms, so 250 ms is generous.
    if (state != jetson_state::kStateSyncWait1 && timer.value() > 250) {
        timeouts++;
        state = jetson_state::kStateSyncWait1;
    }
    timer.reset();  // restart the inter-byte timeout for the next byte

    switch (state) {
      // ── Sync sequence — four fixed bytes mark the start of every packet ──
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

      // ── Payload length (2 bytes, little-endian) ──────────────────────────
      case jetson_state::kStateLength:
        payload_length = (payload_length >> 8) + ((uint16_t)data << 8);
        if (index++ == 1) {
            state = jetson_state::kStateSpare;
            index = 0;
            payload_type = 0;
        }
        break;

      // ── Packet type (2 bytes, little-endian) ──────────────────────────────
      case jetson_state::kStateSpare:
        payload_type = (payload_type >> 8) + ((uint16_t)data << 8);
        if (index++ == 1) {
            state = jetson_state::kStateCrc32;
            index = 0;
            payload_crc32 = 0;
        }
        break;

      // ── Expected CRC32 (4 bytes, little-endian) ───────────────────────────
      case jetson_state::kStateCrc32:
        payload_crc32 = (payload_crc32 >> 8) + ((uint32_t)data << 24);
        if (index++ == 3) {
            state = jetson_state::kStatePayload;
            index = 0;
            calc_crc32 = 0;
        }
        break;

      // ── Payload data — accumulate bytes and run CRC in parallel ──────────
      case jetson_state::kStatePayload:
        if (index < sizeof(payload)) {
            payload.bytes[index] = data;
            index++;
            // Running CRC avoids re-scanning the whole buffer at end-of-packet
            calc_crc32 = crc32(&data, 1, calc_crc32);

            if (index == payload_length) {
                // All payload bytes received — check CRC
                if (payload_crc32 == calc_crc32)
                    state = jetson_state::kStateGoodPacket;
                else
                    state = jetson_state::kStateBadPacket;
                bRecall = true;  // state machine needs one more pass
            }
        } else {
            // Payload overflows buffer — discard packet
            state = jetson_state::kStateBadPacket;
            bRecall = true;
        }
        break;

      // ── Good packet — deserialize AI_RECORD from raw bytes ───────────────
      case jetson_state::kStateGoodPacket:
        if (payload_type == MAP_PACKET_TYPE) {
            AI_RECORD newMap;
            memset(&newMap, 0, sizeof(newMap));

            // ── memcpy 1: detectionCount + POS_RECORD (36 bytes) ─────────
            memcpy(&newMap, &payload.bytes[0], MAP_POS_SIZE);

            // Clamp detection count — never trust unvalidated data from the wire
            if (newMap.detectionCount > MAX_DETECTIONS)
                newMap.detectionCount = MAX_DETECTIONS;

            // ── memcpy 2: DETECTION_OBJECT array ─────────────────────────
            uint32_t det_bytes = sizeof(DETECTION_OBJECT) * newMap.detectionCount;
            memcpy(&newMap.detections,
                   &payload.bytes[MAP_POS_SIZE],
                   det_bytes);

            // ── memcpy 3: extension fields (strategyCode + reserved) ──────
            // Guard: only read if payload is long enough to include the extension.
            // Older Jetson builds without the extension remain forward-compatible.
            uint32_t ext_offset = MAP_POS_SIZE + det_bytes;
            uint32_t ext_size   = sizeof(int32_t)      // strategyCode
                                + sizeof(int32_t) * 3; // reserved[3]

            if (payload_length >= ext_offset + ext_size) {
                memcpy(&newMap.strategyCode,
                       &payload.bytes[ext_offset],
                       sizeof(int32_t));
                memcpy(&newMap.reserved,
                       &payload.bytes[ext_offset + sizeof(int32_t)],
                       sizeof(int32_t) * 3);
            }

            // ── Thread-safe update of the shared last_map ─────────────────
            maplock.lock();
            memcpy(&last_map, &newMap, sizeof(AI_RECORD));
            last_payload_length = payload_length;
            maplock.unlock();
        }

        // Record arrival time using PROS monotonic clock
        last_packet_time = pros::millis();
        packets++;
        state = jetson_state::kStateSyncWait1;
        break;

      // ── Bad packet (CRC failure or buffer overflow) ───────────────────────
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
/** @brief  Background receive task — reads Smart Port bytes, feeds parser    */
/*                                                                            */
/** Serial I/O change from original:                                          */
/**   Original used getchar() on USB CDC stdin (/dev/serial1).               */
/**   This version opens a pros::Serial on the configured Smart Port.        */
/**   read_byte() is non-blocking (returns -1 when no data); the task        */
/**   sleeps 1 ms when the buffer is empty to avoid spinning the CPU,        */
/**   but stays responsive — a packet arrives roughly every 7 ms at 460800.  */
/*----------------------------------------------------------------------------*/
void
jetson::receive_task(void* arg) {
    if (arg == nullptr) return;

    jetson* instance = static_cast<jetson*>(arg);

    // Open Smart Port serial at JETSON_BAUD_RATE baud (port is 1-indexed in PROS)
    pros::Serial serial(instance->_port, JETSON_BAUD_RATE);

    while (true) {
        int rxchar = serial.read_byte();  // returns -1 if no byte available

        if (rxchar >= 0) {
            instance->total_data_received++;
            // parse() returns true when the state machine needs another immediate
            // pass (kStateGoodPacket / kStateBadPacket each need one extra call).
            // Yield between passes so higher-priority tasks can run.
            while (instance->parse(static_cast<uint8_t>(rxchar)))
                pros::delay(1);
        } else {
            // No byte available — sleep rather than busy-wait
            pros::delay(1);
        }
    }
}
