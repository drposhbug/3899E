/*----------------------------------------------------------------------------
 * ai_jetson.h
 *
 * Based on: VEX Robotics VAIC_25_26 ai_jetson.h by James Pearman
 * Modified: Team 3899 VAIRC Push Back (PROS port)
 *
 * Changes from original VEXcode version:
 *   - vex::timer  replaced with a lightweight Timer wrapper (pros::millis)
 *   - vex::mutex  replaced with pros::Mutex
 *   - AI_RECORD   extended: strategyCode + reserved[3] fields added
 *   - Constructor takes Smart Port number (not USB CDC stdin)
 *   - request_map() removed — Jetson pushes asynchronously, no poll needed
 *   - Baud rate updated to 460800
 *   - MAP_POS_SIZE macro added for memcpy offset calculation
 *----------------------------------------------------------------------------*/

#ifndef AI_JETSON_H_
#define AI_JETSON_H_

#include "main.h"    // PROS entry point
#include <stdint.h>

// ── Protocol constants ────────────────────────────────────────────────────────

// Packet-type field value that identifies an AI map payload.
// Must match MAP_PACKET_TYPE in vaic_protocol.py on the Jetson side.
#define MAP_PACKET_TYPE     0x0001

// Maximum number of YOLO detections stored per packet.
// Raise here and in vaic_protocol.py together if more detections are needed.
#define MAX_DETECTIONS      5

// Serial baud rate — must match the jetson_comms.py serial port open call.
#define JETSON_BAUD_RATE    460800

// GPS status flag — set in POS_RECORD.status when the GPS has a valid lock.
#define POS_GPS_CONNECTED   0x01

// ── Strategy codes — dispatched by the Jetson field-state model ───────────────
// V5 state machines key on these values; add new codes symmetrically in both files.
#define STRATEGY_IDLE           0
#define STRATEGY_SCORE_BLUE     1
#define STRATEGY_SCORE_RED      2
#define STRATEGY_DESCORE        3
#define STRATEGY_PARK           4
#define STRATEGY_DEFEND         5
#define STRATEGY_SURVEY         6
#define STRATEGY_SKILLS_SEQ     100

// ── ClassID encoding — matches vaic_protocol.py label ordering ────────────────
// Forward camera (e-CAM25_CUONX) detection classes:
#define CLASS_FWD_BLUE_BLOCK    0
#define CLASS_FWD_RED_BLOCK     1
#define CLASS_FWD_GOAL          2
#define CLASS_FWD_ROBOT         3
#define CLASS_FWD_PARK_ZONE     4
// Survey camera (RealSense D435, Phase 2 only):
#define CLASS_SURVEY_OPP_ROBOT  10
#define CLASS_SURVEY_BLUE_BLOCK 11
#define CLASS_SURVEY_RED_BLOCK  12

// ── Wire-format structs ───────────────────────────────────────────────────────
// All structs are __packed__ so compiler padding cannot misalign fields
// relative to Python struct.pack output on the Jetson.
// Every field is naturally aligned (int32/float = 4 bytes) so packing is free.

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Robot position record sent back from the V5 Brain to the Jetson.
 * NOTE: Jetson zeroes this entire struct — GPS is owned by the V5 Brain.
 * Size: 32 bytes (2×int32_t + 6×float)
 */
typedef struct __attribute__((packed)) {
    int32_t  framecnt;  // increments each Jetson inference frame
    int32_t  status;    // always 0 here (POS_GPS_CONNECTED flag unused on Jetson side)
    float    x, y, z;  // field coordinates in meters (0,0 = field center)
    float    az;        // heading in degrees (azimuth)
    float    el;        // pitch in degrees
    float    rot;       // roll in degrees
} POS_RECORD;

/**
 * Pixel-space bounding box from YOLO inference.
 * Size: 16 bytes (4×int32_t)
 */
typedef struct __attribute__((packed)) {
    int32_t  x, y;          // top-left corner of bounding box (pixels)
    int32_t  width, height; // bounding box dimensions (pixels)
} IMAGE_DETECTION;

/**
 * Field-space position of a detected object.
 * Phase 1: monocular apparent-size depth estimate.
 * Phase 2: RealSense D435 metric depth (replaces estimate).
 * Size: 12 bytes (3×float)
 */
typedef struct __attribute__((packed)) {
    float x;  // field X in meters (0,0 = field center)
    float y;  // field Y in meters
    float z;  // height above field tiles in meters
} MAP_DETECTION;

/**
 * Single object detection from the forward camera YOLO inference.
 * Size: 40 bytes (int32_t + float + float + IMAGE_DETECTION + MAP_DETECTION)
 */
typedef struct __attribute__((packed)) {
    int32_t         classID;        // object class (see CLASS_* defines above)
    float           probability;    // YOLO confidence score, 0.0–1.0
    float           depth;          // distance from camera in meters
    IMAGE_DETECTION screenLocation; // pixel bounding box
    MAP_DETECTION   mapLocation;    // field-space coordinates
} DETECTION_OBJECT;

/**
 * Complete AI map payload — one packet per Jetson push (~40 Hz).
 *
 * Wire layout (must match AIRecord._payload_bytes() in vaic_protocol.py):
 *   int32_t          detectionCount          4 bytes
 *   POS_RECORD       pos                    32 bytes
 *   DETECTION_OBJECT detections[N]          40×N bytes  (N ≤ MAX_DETECTIONS)
 *   int32_t          strategyCode            4 bytes  ← Team 3899 extension
 *   int32_t          reserved[3]            12 bytes  ← Team 3899 extension
 *
 * Extension fields are only copied if payload_length covers their offset,
 * so older Jetson builds without the extension remain compatible.
 *
 * reserved[0]: survey camera motor angle in millidegrees (Phase 2 only)
 * reserved[1–2]: spare
 */
typedef struct __attribute__((packed)) {
    int32_t          detectionCount;
    POS_RECORD       pos;
    DETECTION_OBJECT detections[MAX_DETECTIONS];
    int32_t          strategyCode;   // dispatched by Jetson field-state model
    int32_t          reserved[3];
} AI_RECORD;

// ── Size helpers ──────────────────────────────────────────────────────────────

// Bytes before the detections[] array: detectionCount(4) + POS_RECORD(32) = 36
#define MAP_POS_SIZE  (sizeof(int32_t) + sizeof(POS_RECORD))

/**
 * Packet framing header.
 * The AI_RECORD payload immediately follows the header fields in the stream.
 */
typedef struct __attribute__((packed)) {
    uint8_t   sync[4];   // frame sync bytes: {0xAA, 0x55, 0xCC, 0x33}
    uint16_t  length;    // payload byte count (header not included)
    uint16_t  type;      // packet type: MAP_PACKET_TYPE = 0x0001
    uint32_t  crc32;     // CRC32 of payload bytes only
    AI_RECORD map;       // payload
} map_packet;

#ifdef __cplusplus
} // extern "C"
#endif

// ── C++ class — compiled only in C++ translation units ───────────────────────
#ifdef __cplusplus

namespace ai {

// ── Lightweight timer wrapper ─────────────────────────────────────────────────
// Replaces vex::timer using PROS's pros::millis() monotonic clock.
// Interface is intentionally identical so the jetson class body needs no changes.
class Timer {
public:
    void reset()          { _start = pros::millis(); }
    uint32_t value()      { return pros::millis() - _start; }  // elapsed ms
private:
    uint32_t _start = 0;
};

/**
 * ai::jetson — receives and decodes AI map packets from a Jetson Nano
 * connected to a V5 Smart Port via RS-485 / Podazz serial adapter.
 *
 * The constructor opens the port and starts a background receive task.
 * Packets are decoded by a byte-by-byte state machine and CRC-validated.
 * get_data() provides a thread-safe snapshot of the latest AI_RECORD.
 */
class jetson {
  public:
    /**
     * Open Smart Port serial at JETSON_BAUD_RATE and start the receive task.
     * @param port  V5 Smart Port number matching physical wiring.
     */
    explicit jetson(int32_t port);
    ~jetson();

    int32_t get_packets(void);   // number of good CRC-validated packets received
    int32_t get_errors(void);    // number of CRC failures
    int32_t get_timeouts(void);  // number of inter-byte timeout resets
    int32_t get_total(void);     // total raw bytes received

    /**
     * Thread-safe copy of the latest AI_RECORD into the caller's buffer.
     * Returns the payload length of the last good packet, or 0 if none yet.
     */
    int32_t get_data(AI_RECORD* map);

    /**
     * Convenience: returns only the strategyCode from the latest packet
     * without allocating a full AI_RECORD on the stack.
     */
    int32_t get_strategy(void);

    // request_map() intentionally absent — Jetson pushes continuously.

  private:
    // Four-byte sync sequence that marks the start of every packet.
    enum class sync_byte : uint8_t {
        kSync1 = 0xAA,
        kSync2 = 0x55,
        kSync3 = 0xCC,
        kSync4 = 0x33
    };

    // Byte-by-byte receive state machine states.
    enum class jetson_state {
        kStateSyncWait1 = 0,
        kStateSyncWait2,
        kStateSyncWait3,
        kStateSyncWait4,
        kStateLength,
        kStateSpare,
        kStateCrc32,
        kStatePayload,
        kStateGoodPacket,
        kStateBadPacket,
    };

    int32_t       _port;   // Smart Port number
    jetson_state  state;   // current state-machine state
    int32_t       index;   // byte index within the current field

    Timer         timer;   // inter-byte timeout guard (PROS-based, replaces vex::timer)

    uint32_t  packets;             // good packet counter
    uint32_t  errors;              // CRC failure counter
    uint32_t  timeouts;            // timeout reset counter
    uint16_t  payload_length;      // declared payload length from header
    uint16_t  payload_type;        // packet type from header
    uint32_t  payload_crc32;       // CRC32 from header (expected)
    uint32_t  calc_crc32;          // CRC32 computed over received bytes (actual)
    uint32_t  last_packet_time;    // pros::millis() timestamp of last good packet
    uint32_t  total_data_received; // cumulative raw bytes

    // Receive buffer — 512 bytes > max AI_RECORD (~252 bytes at MAX_DETECTIONS=5).
    union {
        AI_RECORD  map;
        uint8_t    bytes[512];
    } payload;

    pros::Mutex maplock;          // protects last_map / last_payload_length
    AI_RECORD   last_map;         // last successfully decoded packet
    uint32_t    last_payload_length;

    // Feed one byte into the state machine; returns true when a full packet is ready.
    bool parse(uint8_t data);

    // PROS task function: reads bytes from the Smart Port and calls parse().
    static void receive_task(void* arg);

    // CRC32 lookup table and computation function.
    static uint32_t _crc32_table[256];
    static uint32_t crc32(uint8_t* pData, uint32_t numberOfBytes, uint32_t accumulator);
};

} // namespace ai

#endif // __cplusplus

#endif // AI_JETSON_H_
