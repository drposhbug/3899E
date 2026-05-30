/*----------------------------------------------------------------------------
 * jetson_comms.h — Jetson ↔ V5 Brain communication protocol (Team 3899E)
 *
 * Strictly the communication layer:
 *   - Wire-format structs (AI_RECORD, POS_RECORD, DETECTION_OBJECT, etc.)
 *   - Protocol constants (sync bytes, baud rate, packet type, buffer sizes)
 *   - jetson class: binary packet parser, CRC32, background receive task
 *   - Global g_jetson instance declaration
 *
 * Transport: USB (micro-USB on V5 Brain → USB-A on Jetson Orin Nano Super)
 *   Brain appears as /dev/ttyACM1 on Jetson. Brain reads via getchar()
 *   on stdin. Confirmed working at 115200 baud. Competition-legal per VUR12a.
 *
 * Packet format (Jetson → V5, ~40Hz):
 *   [0xAA][0x55][0xCC][0x33][len16][type16][crc32][AI_RECORD payload...]
 *
 * Do NOT include strategy codes or class IDs here — those live in ai.h.
 *----------------------------------------------------------------------------*/

#ifndef JETSON_COMMS_H_
#define JETSON_COMMS_H_

#include "main.h"
#include <stdint.h>

// ── Protocol constants ────────────────────────────────────────────────────────

// Packet type field — identifies an AI map payload.
// Must match MAP_PACKET_TYPE in vaic_protocol.py on Jetson.
#define MAP_PACKET_TYPE     0x0001

// Maximum YOLO detections stored per packet.
// Raise here AND in vaic_protocol.py together.
#define MAX_DETECTIONS      5

// USB serial baud rate — must match jetson_comms.py serial open call.
// Confirmed working at 115200 over USB. 460800 is target for binary protocol.
#define JETSON_BAUD_RATE    115200

// GPS status flag in POS_RECORD.status
#define POS_GPS_CONNECTED   0x01

// Bytes before the detections[] array: detectionCount(4) + POS_RECORD(40) = 44
// NOTE: MAP_POS_SIZE updated from 36 to 44 to account for the three new
// navigation fields added to POS_RECORD (targetTracked, targetDistanceCm,
// obstacleDetected). Must match AIRecord._payload_bytes() in vaic_protocol.py.
#define MAP_POS_SIZE  (sizeof(int32_t) + sizeof(POS_RECORD))

// ── Wire-format structs ───────────────────────────────────────────────────────
// All packed so compiler padding cannot misalign fields relative to
// Python struct.pack output on the Jetson side.

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Robot position record.
 * GPS is owned by the V5 Brain — Jetson zeroes the GPS fields.
 * Navigation state fields (targetTracked, targetDistanceCm, obstacleDetected)
 * are populated by the Jetson and consumed by ai.cpp navigation logic.
 * Size: 40 bytes (was 32 — 8 bytes added for three navigation fields)
 */
typedef struct __attribute__((packed)) {
    int32_t  framecnt;           // Jetson inference frame counter (increments each packet)
    int32_t  status;             // GPS status flags (POS_GPS_CONNECTED; always 0 from Jetson)
    float    x, y, z;            // field position in meters (0,0 = field center)
    float    az;                 // heading in degrees
    float    el;                 // pitch in degrees
    float    rot;                // roll in degrees
    // ── Navigation state (populated by Jetson, read by ai.cpp) ───────────────
    int32_t  targetTracked;      // 1 = Jetson has target lock, 0 = no lock
    float    targetDistanceCm;   // estimated distance to tracked target (cm), -1 if none
    int32_t  obstacleDetected;   // 1 = obstacle detected in forward path, 0 = clear
} POS_RECORD;

/**
 * Pixel-space bounding box from YOLO inference.
 * Size: 16 bytes
 */
typedef struct __attribute__((packed)) {
    int32_t  x, y;          // top-left corner (pixels)
    int32_t  width, height; // bounding box dimensions (pixels)
} IMAGE_DETECTION;

/**
 * Field-space position of a detected object.
 * Phase 1: monocular depth estimate. Phase 2: RealSense D435 metric depth.
 * Size: 12 bytes
 */
typedef struct __attribute__((packed)) {
    float x;  // field X in meters
    float y;  // field Y in meters
    float z;  // height above tiles in meters
} MAP_DETECTION;

/**
 * Single YOLO detection from the forward camera.
 * classID values defined in ai.h (CLASS_FWD_*, CLASS_SURVEY_*).
 * Size: 40 bytes
 */
typedef struct __attribute__((packed)) {
    int32_t         classID;        // object class (see ai.h CLASS_* defines)
    float           probability;    // YOLO confidence 0.0–1.0
    float           depth;          // distance from camera in meters
    IMAGE_DETECTION screenLocation; // pixel bounding box
    MAP_DETECTION   mapLocation;    // field-space coordinates
} DETECTION_OBJECT;

/**
 * Complete AI map payload — one packet per Jetson push (~40Hz).
 *
 * Wire layout (must match AIRecord._payload_bytes() in vaic_protocol.py):
 *   int32_t          detectionCount     4 bytes
 *   POS_RECORD       pos               40 bytes  ← was 32, +8 for nav fields
 *   DETECTION_OBJECT detections[N]     40×N bytes  (N ≤ MAX_DETECTIONS)
 *   int32_t          strategyCode       4 bytes  ← Team 3899 extension
 *   int32_t          reserved[3]       12 bytes  ← Team 3899 extension
 *
 * Extension fields only copied if payload_length covers their offset —
 * older Jetson builds without extension remain compatible.
 *
 * reserved[0]: survey camera motor angle in millidegrees (Phase 2 only)
 * reserved[1–2]: spare
 */
typedef struct __attribute__((packed)) {
    int32_t          detectionCount;
    POS_RECORD       pos;
    DETECTION_OBJECT detections[MAX_DETECTIONS];
    int32_t          strategyCode;  // high-level strategy (see ai.h STRATEGY_*)
    int32_t          reserved[3];
} AI_RECORD;

/**
 * Full framing header + payload — used for reference only.
 * The parser works byte-by-byte; this struct is not cast over the stream.
 */
typedef struct __attribute__((packed)) {
    uint8_t   sync[4];   // {0xAA, 0x55, 0xCC, 0x33}
    uint16_t  length;    // payload byte count
    uint16_t  type;      // MAP_PACKET_TYPE
    uint32_t  crc32;     // CRC32 of payload bytes only
    AI_RECORD map;
} map_packet;

#ifdef __cplusplus
} // extern "C"
#endif

// ── C++ class ─────────────────────────────────────────────────────────────────
#ifdef __cplusplus

namespace comms {

// Lightweight timer using PROS monotonic clock — replaces vex::timer
class Timer {
public:
    void     reset() { _start = pros::millis(); }
    uint32_t value() { return pros::millis() - _start; }
private:
    uint32_t _start = 0;
};

/**
 * comms::jetson — receives and decodes binary AI_RECORD packets from the
 * Jetson Orin Nano Super over USB serial (stdin / getchar()).
 *
 * Constructor starts the JetsonRx background task immediately.
 * All callers use get_data() or get_strategy() for thread-safe access.
 * AI behavior functions in ai.h use the higher-level accessors there.
 */
class jetson {
  public:
    explicit jetson(int32_t port);
    ~jetson();

    int32_t get_packets(void);   // CRC-validated packets received
    int32_t get_errors(void);    // CRC failures
    int32_t get_timeouts(void);  // inter-byte timeout resets
    int32_t get_total(void);     // total raw bytes received

    // Thread-safe copy of latest AI_RECORD. Returns payload length, 0 if none yet.
    int32_t get_data(AI_RECORD* map);

    // Returns only strategyCode from latest packet. Returns 0 if none yet.
    int32_t get_strategy(void);

  private:
    enum class sync_byte : uint8_t {
        kSync1 = 0xAA, kSync2 = 0x55, kSync3 = 0xCC, kSync4 = 0x33
    };

    enum class jetson_state {
        kStateSyncWait1 = 0, kStateSyncWait2, kStateSyncWait3, kStateSyncWait4,
        kStateLength, kStateSpare, kStateCrc32, kStatePayload,
        kStateGoodPacket, kStateBadPacket,
    };

    int32_t      _port;
    jetson_state state;
    int32_t      index;
    Timer        timer;

    uint32_t  packets;
    uint32_t  errors;
    uint32_t  timeouts;
    uint16_t  payload_length;
    uint16_t  payload_type;
    uint32_t  payload_crc32;
    uint32_t  calc_crc32;
    uint32_t  last_packet_time;
    uint32_t  total_data_received;

    union {
        AI_RECORD map;
        uint8_t   bytes[512];  // 512 > max AI_RECORD at MAX_DETECTIONS=5
    } payload;

    pros::Mutex maplock;
    AI_RECORD   last_map;
    uint32_t    last_payload_length;

    bool parse(uint8_t data);
    static void receive_task(void* arg);

    static uint32_t _crc32_table[256];
    static uint32_t crc32(uint8_t* pData, uint32_t numberOfBytes, uint32_t accumulator);
};

} // namespace comms

// Global instance — defined in jetson_comms.cpp, used by ai.cpp and main.cpp
extern comms::jetson g_jetson;
using namespace comms;

// Navigation state atomics — defined in jetson_comms.cpp, extern in ai.cpp
#include <atomic>
extern std::atomic<bool>   jetsonTargetTrackedAtomic;
extern std::atomic<double> jetsonTargetDistanceCmAtomic;
extern std::atomic<bool>   jetsonObstacleDetectedAtomic;

#endif // __cplusplus
#endif // JETSON_COMMS_H_