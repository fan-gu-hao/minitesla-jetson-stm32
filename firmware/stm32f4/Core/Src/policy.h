#ifndef FOUR_TOF_POLICY_H
#define FOUR_TOF_POLICY_H

#include <stdint.h>

/* Bring-up guard only. ESC neutral is not a verified active brake. */
#define POLICY_TOF_COUNT 4U
#define POLICY_STOP_MM 300U
#define POLICY_CLEAR_MM 400U
#define POLICY_CLEAR_HOLD_MS 500U
#define POLICY_TOF_STALE_MS 120U
#define POLICY_COMMAND_STALE_MS 300U
#define POLICY_ESC_MIN 142U
#define POLICY_ESC_MID 152U
#define POLICY_ESC_MAX 160U
#define POLICY_SERVO_MIN 94U
#define POLICY_SERVO_MID 152U
#define POLICY_SERVO_MAX 210U

typedef struct {
    uint32_t last_read_ms; /* Host first-observed-ready time of latest result; never refresh on not-ready. */
    uint16_t mm;
    uint8_t status;       /* ST RangeStatus 0 is valid; 0xFF may encode I2C error. */
    uint8_t seen;
} PolicySensor;

typedef struct {
    uint32_t received_ms;
    uint32_t generation;  /* Increment only after a complete accepted CRC packet. */
    uint8_t seen;
    uint8_t esc;
    uint8_t servo;
} PolicyCommand;

typedef struct {
    PolicySensor sensor[POLICY_TOF_COUNT];
    PolicyCommand command;
} PolicySnapshot;

typedef enum {
    POLICY_LATCHED = 0,
    POLICY_ARMED_NEUTRAL = 1,
    POLICY_RUNNING = 2
} PolicyMode;

enum {
    POLICY_R_UART_UNSEEN = 1UL << 0,
    POLICY_R_UART_STALE = 1UL << 1,
    POLICY_R_UART_RANGE = 1UL << 2,
    POLICY_R_TOF_UNSEEN = 1UL << 3,
    POLICY_R_TOF_STATUS = 1UL << 4,
    POLICY_R_TOF_STALE = 1UL << 5,
    POLICY_R_OBSTACLE = 1UL << 6,
    POLICY_R_WAIT_CLEAR = 1UL << 7,
    POLICY_R_WAIT_NEUTRAL = 1UL << 8,
    POLICY_R_WAIT_NEW_THROTTLE = 1UL << 9
};

typedef struct {
    PolicyMode mode;
    uint32_t clear_since_ms;
    uint32_t consumed_generation;
    uint8_t have_consumed_command;
    uint8_t clear_timer_active;
    uint8_t neutral_seen_during_clear;
} PolicyState;

typedef struct {
    uint8_t esc;
    uint8_t servo;
    uint8_t unseen_mask;
    uint8_t status_mask;
    uint8_t stale_mask;
    uint8_t obstacle_mask;
    uint32_t reasons;
    uint32_t clear_elapsed_ms;
    PolicyMode mode;
    uint8_t neutral_seen_during_clear;
} PolicyOutput;

void policy_init(PolicyState *state);
/* Call from one context only with an atomic, coherent input snapshot.
 * Unsigned age differences permit uint32_t ms rollover. All intervals must
 * remain well below 2^31 ms, and sensor clocks must share the same tick source.
 * No I/O, allocation, waits, locks, or HAL calls occur in this function. */
PolicyOutput policy_step(PolicyState *state, const PolicySnapshot *input,
                         uint32_t now_ms);

#endif
