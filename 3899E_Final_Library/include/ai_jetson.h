/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:     ai_jetson.h                                                 */
/*    Based on:   VEX Robotics VAIC_25_26 ai_jetson.h by James Pearman       */
/*    Modified:   Team 3899 VAIRC Push Back                                   */
/*                                                                            */
/*    Changes from original:                                                  */
/*      - AI_RECORD extended with strategyCode + reserved[3] fields           */
/*      - Constructor takes Smart Port number (not USB CDC stdin)             */
/*      - request_map() removed — async push, no poll needed                  */
/*      - Baud rate updated to 460800                                         */
/*      - MAP_POS_SIZE macro added for memcpy offset calculation              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef AI_JETSON_H_
#define AI_JETSON_H_

#include "vex.h"
#include <stdint.h>

// ── Protocol constants ────────────────────────────────────────────────────────

// Identifies the AI map payload in the packet type field.
// Must match MAP_PACKET_TYPE in vaic_protocol.py on the Jetson.
#define MAP_PACKET_TYPE     0x0001

// Maximum detections stored per packet. Jetson sends no more than this.
// Raise here and in vaic_protocol.py together if more are needed.
#define MAX_DETECTIONS      5

// Serial baud rate — must match jetson_comms.py serial port open call.
#define JETSON_BAUD_RATE    460800

// GPS status flag — set in POS_RECORD.status when GPS has valid lock.
#define POS_GPS_CONNECTED   0x01

// ── Strategy codes — dispatched by Jetson field state model ──────────────────
// V5 state machines key on these; add new codes symmetrically in both files.
#define STRATEGY_IDLE           0
#define STRATEGY_SCORE_BLUE     1
#define STRATEGY_SCORE_RED      2
#define STRATEGY_DESCORE        3
#define STRATEGY_PARK           4
#define STRATEGY_DEFEND         5
#define STRATEGY_SURVEY         6
#define STRATEGY_SKILLS_SEQ     100

// ── ClassID encoding — matches vaic_protocol.py label ordering ───────────────
// Forward camera (e-CAM25_CUONX) detections
#define CLASS_FWD_BLUE_BLOCK    0
#define CLASS_FWD_RED_BLOCK     1
#define CLASS_FWD_GOAL          2
#define CLASS_FWD_ROBOT         3
#define CLASS_FWD_PARK_ZONE     4
// Survey camera (RealSense D435, Phase 2 only)
#define CLASS_SURVEY_OPP_ROBOT  10
#define CLASS_SURVEY_BLUE_BLOCK 11
#define CLASS_SURVEY_RED_BLOCK  12

// ── Wire-format structs ───────────────────────────────────────────────────────
// All structs are __packed__ so compiler padding cannot misalign fields
// relative to Python struct.pack output. Every field is naturally aligned
// (all int32/float = 4 bytes) so packed adds no performance cost here.

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Robot position record.
 * NOTE: Jetson zeroes this entire struct — GPS is owned by the V5 Brain.
 * 32 bytes: 2×int32_t (8) + 6×float (24)
 */
typedef struct __attribute__((packed)) {
    int32_t  framecnt;   // increments each Jetson inference frame
    int32_t  status;     // always 0 (GPS on V5 side; POS_GPS_CONNECTED unused)
    float    x, y, z;   // field coordinates in meters (0,0 = field center)
    float    az;         // heading in degrees
    float    el;         // pitch in degrees
    float    rot;        // roll in degrees
} POS_RECORD;

/**
 * Pixel-space bounding box from YOLO inference.
 * 16 bytes: 4×int32_t
 */
typedef struct __attribute__((packed)) {
    int32_t  x, y;          // top-left corner (pixels)
    int32_t  width, height; // bounding box size (pixels)
} IMAGE_DETECTION;

/**
 * Field-space position of detected object.
 * Phase 1: monocular apparent-size depth estimation.
 * Phase 2: RealSense D435 metric depth (replaces estimate).
 * 12 bytes: 3×float
 */
typedef struct __attribute__((packed)) {
    float x;  // field X in meters (0,0 = field center)
    float y;  // field Y in meters
    float z;  // height above field tiles in meters
} MAP_DETECTION;

/**
 * Single object detection from forward camera YOLO inference.
 * 40 bytes: int32_t(4) + float(4) + float(4) + IMAGE_DETECTION(16) + MAP_DETECTION(12)
 */
typedef struct __attribute__((packed)) {
    int32_t         classID;        // object class (see CLASS_* defines above)
    float           probability;    // YOLO confidence 0.0-1.0
    float           depth;          // meters from camera (monocular or RealSense)
    IMAGE_DETECTION screenLocation; // pixel bounding box
    MAP_DETECTION   mapLocation;    // field-space coordinates
} DETECTION_OBJECT;

/**
 * Complete AI map payload — one packet per Jetson push (~40 Hz).
 *
 * Wire layout (must match AIRecord._payload_bytes() in vaic_protocol.py):
 *   int32_t          detectionCount          4 bytes
 *   POS_RECORD       pos                    32 bytes
 *   DETECTION_OBJECT detections[N]          40xN bytes  (N <= MAX_DETECTIONS)
 *   int32_t          strategyCode            4 bytes  <- extension
 *   int32_t          reserved[3]            12 bytes  <- extension
 *
 * Extension fields are guarded in ai_jetson.cpp: only copied if payload_length
 * covers their offset, so older Jetson builds without extension remain compatible.
 *
 * reserved[0]: survey camera motor angle in millidegrees (Phase 2)
 * reserved[1-2]: spare
 */
typedef struct __attribute__((packed)) {
    int32_t          detectionCount;
    POS_RECORD       pos;
    DETECTION_OBJECT detections[MAX_DETECTIONS];
    int32_t          strategyCode;
    int32_t          reserved[3];
} AI_RECORD;

// ── Size helpers ──────────────────────────────────────────────────────────────

// Bytes before the detections array: detectionCount(4) + POS_RECORD(32) = 36
#define MAP_POS_SIZE  (sizeof(int32_t) + sizeof(POS_RECORD))

// Packet framing header. Payload (AI_RECORD) follows the header fields.
typedef struct __attribute__((packed)) {
    uint8_t   sync[4];   // {0xAA, 0x55, 0xCC, 0x33}
    uint16_t  length;    // payload byte count (header not included)
    uint16_t  type;      // MAP_PACKET_TYPE = 0x0001
    uint32_t  crc32;     // CRC32 of payload bytes only
    AI_RECORD map;       // payload
} map_packet;

#ifdef __cplusplus
} // extern "C"
#endif

// ── C++ class — compiled only from C++ translation units ─────────────────────

#ifdef __cplusplus

namespace ai {

class jetson {
  public:
    /**
     * Opens Smart Port serial at JETSON_BAUD_RATE and starts receive task.
     * @param port  V5 Smart Port number matching physical RS485/Podazz wiring.
     */
    explicit jetson(int32_t port);
    ~jetson();

    int32_t get_packets(void);   // good packets received
    int32_t get_errors(void);    // CRC failures
    int32_t get_timeouts(void);  // inter-byte timeout resets
    int32_t get_total(void);     // total raw bytes received

    /**
     * Thread-safe copy of the latest AI_RECORD into caller's buffer.
     * Returns payload length of last good packet, or 0 if none received yet.
     */
    int32_t get_data(AI_RECORD *map);

    /**
     * Returns strategyCode from the latest packet.
     * Convenience wrapper; avoids allocating a full AI_RECORD on the stack.
     */
    int32_t get_strategy(void);

    // request_map() intentionally absent — Jetson pushes continuously.

  private:
    enum class sync_byte : uint8_t {
        kSync1 = 0xAA,
        kSync2 = 0x55,
        kSync3 = 0xCC,
        kSync4 = 0x33
    };

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

    int32_t       _port;
    jetson_state  state;
    int32_t       index;
    vex::timer    timer;

    uint32_t  packets;
    uint32_t  errors;
    uint32_t  timeouts;
    uint16_t  payload_length;
    uint16_t  payload_type;
    uint32_t  payload_crc32;
    uint32_t  calc_crc32;
    uint32_t  last_packet_time;
    uint32_t  total_data_received;

    // Receive buffer — 512 bytes >> max AI_RECORD (~252 bytes at MAX_DETECTIONS=5)
    union {
        AI_RECORD  map;
        uint8_t    bytes[512];
    } payload;

    vex::mutex  maplock;
    AI_RECORD   last_map;
    uint32_t    last_payload_length;

    bool    parse(uint8_t data);
    static  int receive_task(void *arg);

    static uint32_t _crc32_table[256];
    static uint32_t crc32(uint8_t *pData, uint32_t numberOfBytes, uint32_t accumulator);
};

} // namespace ai

#endif // __cplusplus

#endif // AI_JETSON_H_