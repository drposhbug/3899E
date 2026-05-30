/*----------------------------------------------------------------------------
 * jetson_comms.cpp — Jetson ↔ V5 Brain communication protocol (Team 3899E)
 *
 * Implements comms::jetson: binary packet parser, CRC32, background receive
 * task, and the global g_jetson instance.
 *
 * Transport: USB stdin via getchar(). getchar() is used (not fgets) because
 * binary payloads can contain 0x0A bytes that fgets treats as line endings.
 * getchar() blocks until a byte arrives — no busy-wait needed.
 *
 * Based on VEX Robotics VAIC_25_26 ai_jetson.cpp by James Pearman.
 * Parser state machine and CRC32 algorithm unchanged from original.
 *----------------------------------------------------------------------------*/
#include "jetson_comms.h"
#include <cstring>
#include <atomic>

using namespace comms;

// Static CRC32 table — shared across all instances, built on first use
uint32_t jetson::_crc32_table[256];

// Global instance — constructor starts JetsonRx task before main() runs
comms::jetson g_jetson(0);  // port 0 = placeholder; USB needs no port number

// ── Navigation state atomics ──────────────────────────────────────────────────
// Updated by receive_task when a good packet arrives.
// Read by ai.cpp via jetsonTargetTracked(), jetsonTargetDistance(),
// jetsonObstacleDetected() — declared extern in jetson_comms.h and ai.cpp.
std::atomic<bool>   jetsonTargetTrackedAtomic(false);
std::atomic<double> jetsonTargetDistanceCmAtomic(-1.0);
std::atomic<bool>   jetsonObstacleDetectedAtomic(false);

/*----------------------------------------------------------------------------*/
jetson::jetson(int32_t port) : _port(port) {
    state               = jetson_state::kStateSyncWait1;
    packets             = 0;
    errors              = 0;
    timeouts            = 0;
    total_data_received = 0;
    last_payload_length = 0;
    last_packet_time    = 0;
    memset(&last_map, 0, sizeof(last_map));

    // Background task reads stdin bytes and feeds the parser.
    // TASK_PRIORITY_DEFAULT+2 keeps it responsive during motion loops.
    pros::Task(receive_task, static_cast<void*>(this),
               TASK_PRIORITY_DEFAULT + 2, TASK_STACK_DEPTH_DEFAULT, "JetsonRx");
}

jetson::~jetson() {}

/*----------------------------------------------------------------------------*/
int32_t jetson::get_packets()  { return packets; }
int32_t jetson::get_errors()   { return errors; }
int32_t jetson::get_timeouts() { return timeouts; }
int32_t jetson::get_total()    { return total_data_received; }

/*----------------------------------------------------------------------------*/
// Thread-safe copy of the latest AI_RECORD into caller's buffer.
// Returns payload length of last good packet, 0 if none received yet.
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
// Returns only strategyCode from latest packet without allocating AI_RECORD.
int32_t
jetson::get_strategy(void) {
    maplock.lock();
    int32_t code = last_map.strategyCode;
    maplock.unlock();
    return code;
}

/*----------------------------------------------------------------------------*/
// CRC32 — identical algorithm to original James Pearman implementation
// and vaic_protocol.py. Lookup table built lazily on first call.
uint32_t
jetson::crc32(uint8_t* pData, uint32_t numberOfBytes, uint32_t accumulator) {
    uint32_t i, j;
    const uint32_t POLYNOMIAL_CRC32 = 0x04C11DB7;

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
// Byte-by-byte packet state machine. Returns true when an extra pass is needed
// (kStateGoodPacket and kStateBadPacket both require one additional call).
// 250ms inter-byte timeout resets the machine on a stalled/corrupt packet.
bool
jetson::parse(uint8_t data) {
    bool bRecall = false;

    if (state != jetson_state::kStateSyncWait1 && timer.value() > 250) {
        timeouts++;
        state = jetson_state::kStateSyncWait1;
    }
    timer.reset();

    switch (state) {
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
            state          = jetson_state::kStateLength;
            index          = 0;
            payload_length = 0;
        }
        break;

      case jetson_state::kStateLength:
        payload_length = (payload_length >> 8) + ((uint16_t)data << 8);
        if (index++ == 1) {
            state        = jetson_state::kStateSpare;
            index        = 0;
            payload_type = 0;
        }
        break;

      case jetson_state::kStateSpare:
        payload_type = (payload_type >> 8) + ((uint16_t)data << 8);
        if (index++ == 1) {
            state         = jetson_state::kStateCrc32;
            index         = 0;
            payload_crc32 = 0;
        }
        break;

      case jetson_state::kStateCrc32:
        payload_crc32 = (payload_crc32 >> 8) + ((uint32_t)data << 24);
        if (index++ == 3) {
            state      = jetson_state::kStatePayload;
            index      = 0;
            calc_crc32 = 0;
        }
        break;

      case jetson_state::kStatePayload:
        if (index < sizeof(payload)) {
            payload.bytes[index] = data;
            index++;
            calc_crc32 = crc32(&data, 1, calc_crc32);  // running CRC

            if (index == payload_length) {
                state   = (payload_crc32 == calc_crc32)
                          ? jetson_state::kStateGoodPacket
                          : jetson_state::kStateBadPacket;
                bRecall = true;
            }
        } else {
            state   = jetson_state::kStateBadPacket;
            bRecall = true;
        }
        break;

      case jetson_state::kStateGoodPacket:
        if (payload_type == MAP_PACKET_TYPE) {
            AI_RECORD newMap;
            memset(&newMap, 0, sizeof(newMap));

            // Copy detectionCount + POS_RECORD
            memcpy(&newMap, &payload.bytes[0], MAP_POS_SIZE);

            if (newMap.detectionCount > MAX_DETECTIONS)
                newMap.detectionCount = MAX_DETECTIONS;

            // Copy DETECTION_OBJECT array
            uint32_t det_bytes = sizeof(DETECTION_OBJECT) * newMap.detectionCount;
            memcpy(&newMap.detections, &payload.bytes[MAP_POS_SIZE], det_bytes);

            // Copy extension fields (strategyCode + reserved) if present.
            // Guard ensures older Jetson builds without extension are compatible.
            uint32_t ext_offset = MAP_POS_SIZE + det_bytes;
            uint32_t ext_size   = sizeof(int32_t) + sizeof(int32_t) * 3;

            if (payload_length >= ext_offset + ext_size) {
                memcpy(&newMap.strategyCode,
                       &payload.bytes[ext_offset],
                       sizeof(int32_t));
                memcpy(&newMap.reserved,
                       &payload.bytes[ext_offset + sizeof(int32_t)],
                       sizeof(int32_t) * 3);
            }

            maplock.lock();
            memcpy(&last_map, &newMap, sizeof(AI_RECORD));
            last_payload_length = payload_length;
            maplock.unlock();

            // Update navigation atomics from latest packet fields.
            // These are read by ai.cpp between waypoints — no lock needed (atomic).
            jetsonTargetTrackedAtomic.store(newMap.pos.targetTracked != 0);
            jetsonTargetDistanceCmAtomic.store(static_cast<double>(newMap.pos.targetDistanceCm));
            jetsonObstacleDetectedAtomic.store(newMap.pos.obstacleDetected != 0);
        }

        last_packet_time = pros::millis();
        packets++;
        state = jetson_state::kStateSyncWait1;
        break;

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
// Background receive task — reads bytes from USB stdin and feeds the parser.
// getchar() blocks until a byte arrives; no busy-wait or sleep needed.
// Binary payloads can contain 0x0A — getchar() handles this correctly unlike
// fgets() which would treat 0x0A as end-of-line and truncate the packet.
void
jetson::receive_task(void* arg) {
    if (arg == nullptr) return;

    jetson* instance = static_cast<jetson*>(arg);

    while (true) {
        int rxchar = getchar();  // blocks on empty buffer; returns -1 on EOF

        if (rxchar >= 0) {
            instance->total_data_received++;
            // parse() returns true when it needs one more pass
            while (instance->parse(static_cast<uint8_t>(rxchar)))
                pros::delay(1);
        }
    }
}