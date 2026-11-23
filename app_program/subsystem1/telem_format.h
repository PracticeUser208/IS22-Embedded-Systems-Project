/*
 *  telem_format.h
 *  Subsystem 1 (Buddy 1) -- application data formats for telemetry,
 *  heartbeat, remote commands and ACKs.
 *
 *  Change Requirement 1 (minimum-bit data representation):
 *    - Inside the car (telem_q, remote_cmd_q) data uses C BIT-FIELDS sized
 *      to each field's valid range.
 *    - On the wire (MQTT) data uses EXPLICIT MASKS AND SHIFTS (bitpack.c),
 *      because bit-field layout is compiler-defined.
 *
 *  Version 2: fields aligned with the Project Write-up (6 IMU motion
 *  events, full obstacle profile: angle, left/right clearance, avoidance
 *  plan; barcode IR status; 14-bit distance).
 *
 *  ALL widths below are PROVISIONAL (marked TBC) until each buddy confirms
 *  the range and precision of their own fields. To change a field:
 *    1. change its TF_*_BITS (and range macros);
 *    2. update the enum if the value list changed;
 *    3. bump TF_VERSION;
 *    4. rebuild the PC tests.
 *  The _Static_asserts stop the build if an enum no longer fits its field.
 *
 *  Kernel-free (stdint only) so it builds on a PC for unit tests.
 */

#ifndef TELEM_FORMAT_H
#define TELEM_FORMAT_H

#include <stdint.h>

/* ===================================================================
 *  Format version: sent in every car -> PC frame (except the ACK).
 *  Bump it whenever any width, order or enum on the wire changes.
 * =================================================================== */
#define TF_VERSION              2u
#define TF_VERSION_BITS         3u

/* ===================================================================
 *  MQTT topics
 * =================================================================== */
#define TF_TOPIC_TM             "is22/tm"     /* telemetry,  car -> PC */
#define TF_TOPIC_HB             "is22/hb"     /* heartbeat,  car -> PC */
#define TF_TOPIC_ACK            "is22/ack"    /* command ACK, car -> PC */
#define TF_TOPIC_CMD            "is22/cmd"    /* command,    PC -> car */
#define TF_TOPIC_BC             "is22/bc"     /* barcode read, car -> PC */
#define TF_TOPIC_IR             "is22/ir"     /* side sensor trace, car -> PC */

/* is22/ir payload (3 Nov, debugging the barcode reading): the side
 * sensor's raw ADC at every line-task sample (10 ms), 2 messages a second.
 *   byte 0      line state: 0 standing, 1 following, 2 other
 *   byte 1      sequence number (+1 per message)
 *   byte 2      calibrated white / 16 (0 = not calibrated)
 *   byte 3      calibrated black / 16
 *   bytes 4..   TF_IR_TRACE_N samples, raw / 16 (0..255), oldest first */
#define TF_IR_TRACE_N           50u
#define TF_IR_HDR_LEN           4u
#define TF_IR_LEN               (TF_IR_HDR_LEN + TF_IR_TRACE_N)

/* is22/bc payload: one message per barcode read (or failed read).  Not
 * bit-packed; the telemetry frame is unchanged.
 *   byte 0      barcode_t the car made of it (NONE = not read)
 *   byte 1      bit 0: the car was standing (pushed by hand: not acted on)
 *               bit 1: reading abandoned (a bar or space too long, ...)
 *               bit 2: read on the second look (bc_rec.c: the recording
 *                      classified again with the final calibration)
 *               bit 3: bytes 7-14 hold the side sensor's levels (always
 *                      set since 3 Nov)
 *               bit 4: unused (was the bar-shape reader, removed 8 Nov)
 *               bit 5: the side sensor dropped out during it (loose
 *                      wire, 4 Nov)
 *   byte 2      how many bars + spaces were read
 *   bytes 3-6   the characters between the '*'s, NUL-padded ("" = none)
 *   bytes 7-14  side sensor raw ADC, 4 x uint16 little-endian: lightest
 *               and darkest in the recording (paper, bars), and its
 *               calibrated white and black (usually the black tape)
 *   bytes 15..  each bar/space width in 10 ms units, 255 = 2.55 s or more;
 *               the first is the first bar the sensor met (at most
 *               TF_BC_WIDTHS_MAX)                                      */
#define TF_BC_HDR_LEN           15u
#define TF_BC_TEXT_LEN          4u
#define TF_BC_WIDTHS_MAX        59u
#define TF_BC_LEN_MAX           (TF_BC_HDR_LEN + TF_BC_WIDTHS_MAX)
#define TF_BC_STANDING          0x01u
#define TF_BC_ABANDONED         0x02u
#define TF_BC_REREAD            0x04u
#define TF_BC_LEVELS            0x08u
#define TF_BC_DROPOUTS          0x20u

/* ===================================================================
 *  Enumerations. Each ends with *_COUNT so the asserts can check widths.
 * =================================================================== */

/* System (Vehicle Controller) state -- TBC with VC owner */
typedef enum
{
    SYS_INITIALISING = 0,
    SYS_IDLE_READY,
    SYS_PATROLLING,
    SYS_PAUSED,
    SYS_HANDLING_BARCODE,
    SYS_HANDLE_HUMP,
    SYS_OBSTACLE_AVOIDANCE,
    SYS_LINE_RECOVERY,
    SYS_FAULT_ESTOP,
    SYS_STARTUP_ERROR,
    SYS_STATE_COUNT
} sys_state_t;

/* Motion Control drive mode -- TBC with Buddy 2 */
typedef enum
{
    DRIVE_FOLLOW_LINE = 0,
    DRIVE_OPEN_LOOP,
    DRIVE_MODE_COUNT
} drive_mode_t;

/* Line and Barcode state -- TBC with Buddy 3 */
typedef enum
{
    LINE_INIT = 0,
    LINE_FOLLOWING,
    LINE_BARCODE_SCAN,
    LINE_DECODE,
    LINE_SUSPENDED,
    LINE_SEARCH,
    LINE_FAULT,
    LINE_STATE_COUNT
} line_state_t;

/* Last decoded barcode -- TBC with Buddy 3 */
typedef enum
{
    BARCODE_NONE = 0,
    BARCODE_LEFT,
    BARCODE_RIGHT,
    BARCODE_STRAIGHT,
    BARCODE_UTURN,
    BARCODE_INVALID,
    BARCODE_COUNT
} barcode_t;

/* IMU motion events (Write-up, Buddy 4) -- TBC with Buddy 4 */
typedef enum
{
    MOTION_STATIONARY = 0,
    MOTION_ACCELERATING,
    MOTION_TURNING,
    MOTION_CLIMBING_HUMP,
    MOTION_DESCENDING_HUMP,
    MOTION_LIFTED,          /* 16 Nov: was MOTION_IMPACT (same value, 5): the car
                               was lifted or overturned */
    MOTION_STEADY,          /* added for Buddy 4's motion classes (moving at a
                               steady speed). Appended: values 0..5 and the field
                               width are unchanged, so v2 frames decode as before */
    MOTION_COUNT
} motion_t;

/* Avoidance plan (Write-up, Buddy 5 Stage 4) -- TBC with Buddy 5 */
typedef enum
{
    OBST_PLAN_NONE = 0,         /* no obstacle being handled        */
    OBST_PLAN_STOP,
    OBST_PLAN_CONTINUE,
    OBST_PLAN_TURN_LEFT,
    OBST_PLAN_TURN_RIGHT,
    OBST_PLAN_REVERSE,          /* reverse and reattempt            */
    OBST_PLAN_BLOCKED,          /* no safe side (NO_PATH)           */
    OBST_PLAN_COUNT
} obst_plan_t;

/* Obstacle Scan state -- TBC with Buddy 5 */
typedef enum
{
    OBST_PATROLLING = 0,
    OBST_DETECTED,
    OBST_COARSE_SCAN,
    OBST_FINE_SCAN,
    OBST_PROFILE_PLAN,
    OBST_BYPASSING,
    OBST_REACQUIRE,
    OBST_BLOCKED,
    OBST_SENSOR_FAULT,
    OBST_STATE_COUNT
} obst_state_t;

/* Remote command codes (0, 6, 7 are invalid on the wire) */
typedef enum
{
    CMD_NONE = 0,
    CMD_START,
    CMD_STOP,
    CMD_PAUSE,
    CMD_RESUME,
    CMD_RESET,
    CMD_CODE_COUNT
} cmd_code_t;

/* ACK result */
typedef enum
{
    ACK_FORWARDED = 0,      /* passed to the Vehicle Controller       */
    ACK_DUPLICATE,          /* same code+seq as last one, not resent  */
    ACK_INVALID,            /* wrong length or code                   */
    ACK_QUEUE_FULL,         /* remote_cmd_q full, command dropped     */
    ACK_RESULT_COUNT
} ack_result_t;

/* ===================================================================
 *  Bit masks: one bit per condition (Change Requirement: "individual
 *  bits within a shared fault bitmask").  FAULT_xxx and ESTOP_xxx now
 *  come from the team's shared header, so telemetry, Motion Control,
 *  the IMU and Obstacle Scan all use the same values.  Same bits as
 *  before.  Still plain #defines: the PC tests build unchanged.
 * =================================================================== */
#include "../shared_bits.h"

/* ===================================================================
 *  Field widths (bits) and ranges. TBC = provisional.
 * =================================================================== */
#define TF_SEQ_BITS             8u      /* frame counter, wraps            */
#define TF_SYS_STATE_BITS       4u      /* TBC VC                          */
#define TF_DRIVE_MODE_BITS      1u      /* TBC Buddy 2                     */
#define TF_SPEED_BITS           7u      /* TBC Buddy 2, signed cm/s        */
#define TF_SPEED_MIN            (-64)
#define TF_SPEED_MAX            63
#define TF_ENC_BITS             12u     /* TBC Buddy 2, count mod 4096     */
#define TF_DIST_BITS            14u     /* TBC Buddy 2, cm (0-163 m)       */
#define TF_DIST_MAX             16383u
#define TF_IR_BITS              1u      /* TBC Buddy 3, 1 = on the line    */
#define TF_LINE_STATE_BITS      3u      /* TBC Buddy 3                     */
#define TF_BARCODE_BITS         3u      /* TBC Buddy 3                     */
#define TF_HUMP_MM_BITS         7u      /* TBC Buddy 4, mm                 */
#define TF_HUMP_MM_MAX          127u
#define TF_HUMP_COUNT_BITS      4u      /* TBC Buddy 4                     */
#define TF_HUMP_COUNT_MAX       15u
#define TF_MOTION_BITS          3u      /* TBC Buddy 4, 6 events           */
#define TF_OBST_DIST_BITS       9u      /* TBC Buddy 5, cm                 */
#define TF_OBST_DIST_MAX        510u
#define TF_OBST_NO_ECHO         511u    /* sentinel: no echo / nothing ahead */
#define TF_OBST_WIDTH_BITS      6u      /* TBC Buddy 5, cm                 */
#define TF_OBST_WIDTH_MAX       63u
#define TF_OBST_ANGLE_BITS      5u      /* TBC Buddy 5, servo step index   */
#define TF_OBST_ANGLE_MIN_DEG   30u     /* index 0 = 30 deg                */
#define TF_OBST_ANGLE_STEP_DEG  5u      /* index n = 30 + 5n deg (max 150) */
#define TF_OBST_ANGLE_MAX_IDX   24u     /* 150 deg                         */
#define TF_OBST_ANGLE_NONE      31u     /* sentinel: no bearing yet        */
#define TF_CLEAR_BITS           6u      /* TBC Buddy 5, cm                 */
#define TF_CLEAR_MAX            63u     /* 63 = 63 cm or more              */
#define TF_OBST_PLAN_BITS       3u      /* TBC Buddy 5                     */
#define TF_OBST_STATE_BITS      4u      /* TBC Buddy 5                     */
#define TF_FAULT_BITS           8u      /* TBC VC                          */
#define TF_ESTOP_BITS           3u      /* TBC Buddy 4 / VC                */

/* Heartbeat-only fields (Buddy 1 owns these, not TBC) */
#define TF_UPTIME_BITS          16u     /* seconds, saturates at 65535     */
#define TF_DROPS_BITS           8u      /* telemetry frames dropped        */
#define TF_RECONNECT_BITS       4u      /* MQTT sessions regained          */

/* Command / ACK (Buddy 1 owns these, not TBC) */
#define TF_CMD_CODE_BITS        3u
#define TF_CMD_SEQ_BITS         5u
#define TF_ACK_RESULT_BITS      2u

/* ===================================================================
 *  Wire frame sizes, derived from the widths above. Keep in the same
 *  order as the encoder in telem_codec.c.
 * =================================================================== */
#define TF_TM_BITS  (TF_VERSION_BITS + TF_SEQ_BITS + TF_SYS_STATE_BITS +   \
                     TF_DRIVE_MODE_BITS + (2u * TF_SPEED_BITS) +           \
                     (2u * TF_ENC_BITS) + TF_DIST_BITS + (3u * TF_IR_BITS) + \
                     TF_LINE_STATE_BITS + TF_BARCODE_BITS +                \
                     (2u * TF_HUMP_MM_BITS) + TF_HUMP_COUNT_BITS +         \
                     TF_MOTION_BITS + TF_OBST_DIST_BITS +                  \
                     TF_OBST_WIDTH_BITS + TF_OBST_ANGLE_BITS +             \
                     (2u * TF_CLEAR_BITS) + TF_OBST_PLAN_BITS +            \
                     TF_OBST_STATE_BITS + TF_FAULT_BITS + TF_ESTOP_BITS)

#define TF_HB_BITS  (TF_VERSION_BITS + TF_SEQ_BITS + TF_SYS_STATE_BITS +   \
                     TF_FAULT_BITS + TF_UPTIME_BITS + TF_DROPS_BITS +      \
                     TF_RECONNECT_BITS)

#define TF_CMD_BITS (TF_CMD_CODE_BITS + TF_CMD_SEQ_BITS)
#define TF_ACK_BITS (TF_CMD_BITS + TF_ACK_RESULT_BITS)

#define TF_BYTES(bits)          (((bits) + 7u) / 8u)
#define TF_TM_LEN               TF_BYTES(TF_TM_BITS)    /* 19 with TBC widths */
#define TF_HB_LEN               TF_BYTES(TF_HB_BITS)    /*  7 */
#define TF_CMD_LEN              TF_BYTES(TF_CMD_BITS)   /*  1 */
#define TF_ACK_LEN              TF_BYTES(TF_ACK_BITS)   /*  2 */

/* ===================================================================
 *  In-car messages (C bit-fields). Not sent on the wire as-is.
 * =================================================================== */

/* telem_q message: Vehicle Controller -> Telemetry task, 1 Hz.
   version and seq are NOT here: the Telemetry task adds them. */
typedef struct
{
    /* word 1: 30 bits */
    uint32_t sys_state   : TF_SYS_STATE_BITS;
    uint32_t drive_mode  : TF_DRIVE_MODE_BITS;
    int32_t  speed_l     : TF_SPEED_BITS;       /* signed */
    int32_t  speed_r     : TF_SPEED_BITS;       /* signed */
    uint32_t ir_left     : TF_IR_BITS;
    uint32_t ir_center   : TF_IR_BITS;
    uint32_t ir_barcode  : TF_IR_BITS;
    uint32_t line_state  : TF_LINE_STATE_BITS;
    uint32_t barcode     : TF_BARCODE_BITS;

    /* word 2: 31 bits */
    uint32_t enc_l       : TF_ENC_BITS;
    uint32_t enc_r       : TF_ENC_BITS;
    uint32_t motion      : TF_MOTION_BITS;
    uint32_t hump_count  : TF_HUMP_COUNT_BITS;

    /* word 3: 30 bits */
    uint32_t dist_cm     : TF_DIST_BITS;
    uint32_t obst_dist   : TF_OBST_DIST_BITS;
    uint32_t hump_last   : TF_HUMP_MM_BITS;

    /* word 4: 30 bits */
    uint32_t hump_peak   : TF_HUMP_MM_BITS;
    uint32_t obst_width  : TF_OBST_WIDTH_BITS;
    uint32_t obst_angle  : TF_OBST_ANGLE_BITS;
    uint32_t clear_l     : TF_CLEAR_BITS;
    uint32_t clear_r     : TF_CLEAR_BITS;

    /* word 5: 18 bits */
    uint32_t obst_plan   : TF_OBST_PLAN_BITS;
    uint32_t obst_state  : TF_OBST_STATE_BITS;
    uint32_t fault       : TF_FAULT_BITS;
    uint32_t estop       : TF_ESTOP_BITS;
} telem_snapshot_t;

/* remote_cmd_q message: Telemetry task -> Vehicle Controller */
typedef struct
{
    uint8_t code : TF_CMD_CODE_BITS;    /* cmd_code_t, 1..5 */
    uint8_t seq  : TF_CMD_SEQ_BITS;
} remote_cmd_t;

/* ===================================================================
 *  Compile-time checks: build fails if a value list outgrows its field
 *  or a frame breaks NFR7 (<= 256 bytes per message).
 * =================================================================== */
_Static_assert(SYS_STATE_COUNT  <= (1u << TF_SYS_STATE_BITS),  "sys_state field too small");
_Static_assert(DRIVE_MODE_COUNT <= (1u << TF_DRIVE_MODE_BITS), "drive_mode field too small");
_Static_assert(LINE_STATE_COUNT <= (1u << TF_LINE_STATE_BITS), "line_state field too small");
_Static_assert(BARCODE_COUNT    <= (1u << TF_BARCODE_BITS),    "barcode field too small");
_Static_assert(MOTION_COUNT     <= (1u << TF_MOTION_BITS),     "motion field too small");
_Static_assert(OBST_PLAN_COUNT  <= (1u << TF_OBST_PLAN_BITS),  "obst_plan field too small");
_Static_assert(TF_OBST_ANGLE_NONE < (1u << TF_OBST_ANGLE_BITS), "angle sentinel does not fit");
_Static_assert(TF_OBST_ANGLE_MAX_IDX < TF_OBST_ANGLE_NONE,     "angle index collides with sentinel");
_Static_assert(OBST_STATE_COUNT <= (1u << TF_OBST_STATE_BITS), "obst_state field too small");
_Static_assert(CMD_CODE_COUNT   <= (1u << TF_CMD_CODE_BITS),   "cmd code field too small");
_Static_assert(ACK_RESULT_COUNT <= (1u << TF_ACK_RESULT_BITS), "ack result field too small");
_Static_assert(TF_OBST_NO_ECHO  <  (1u << TF_OBST_DIST_BITS),  "no-echo sentinel does not fit");
_Static_assert(TF_TM_LEN  <= 256u, "NFR7: telemetry frame > 256 B");
_Static_assert(TF_HB_LEN  <= 256u, "NFR7: heartbeat frame > 256 B");
_Static_assert(sizeof(remote_cmd_t) == 1u, "remote_cmd_t must stay 1 byte");
_Static_assert(sizeof(telem_snapshot_t) <= 20u, "telem_q message grew past 20 bytes");

#endif /* TELEM_FORMAT_H */
