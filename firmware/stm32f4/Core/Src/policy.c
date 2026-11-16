#include "policy.h"

void policy_init(PolicyState *state)
{
    state->mode = POLICY_LATCHED;
    state->clear_since_ms = 0U;
    state->consumed_generation = 0U;
    state->have_consumed_command = 0U;
    state->clear_timer_active = 0U;
    state->neutral_seen_during_clear = 0U;
}

PolicyOutput policy_step(PolicyState *state, const PolicySnapshot *input,
                         uint32_t now_ms)
{
    PolicyOutput out = {0};
    const PolicyCommand *cmd = &input->command;
    uint8_t fresh_command = 0U;
    uint8_t new_command = 0U;
    uint8_t all_clear = 1U;
    uint32_t fault_reasons = 0U;
    unsigned i;

    out.esc = POLICY_ESC_MID;
    out.servo = POLICY_SERVO_MID;

    if (!cmd->seen) {
        fault_reasons |= POLICY_R_UART_UNSEEN;
    } else {
        new_command = (uint8_t)(!state->have_consumed_command ||
                               cmd->generation != state->consumed_generation);
        state->consumed_generation = cmd->generation;
        state->have_consumed_command = 1U;
        if ((uint32_t)(now_ms - cmd->received_ms) > POLICY_COMMAND_STALE_MS) {
            fault_reasons |= POLICY_R_UART_STALE;
        } else {
            fresh_command = 1U;
            if (cmd->servo >= POLICY_SERVO_MIN && cmd->servo <= POLICY_SERVO_MAX)
                out.servo = cmd->servo;
            if (cmd->esc < POLICY_ESC_MIN || cmd->esc > POLICY_ESC_MAX ||
                cmd->servo < POLICY_SERVO_MIN || cmd->servo > POLICY_SERVO_MAX)
                fault_reasons |= POLICY_R_UART_RANGE;
        }
    }

    for (i = 0U; i < POLICY_TOF_COUNT; ++i) {
        const PolicySensor *sensor = &input->sensor[i];
        uint8_t bit = (uint8_t)(1U << i);
        if (!sensor->seen) {
            out.unseen_mask |= bit;
            fault_reasons |= POLICY_R_TOF_UNSEEN;
            all_clear = 0U;
            continue;
        }
        if (sensor->status != 0U) {
            out.status_mask |= bit;
            fault_reasons |= POLICY_R_TOF_STATUS;
            all_clear = 0U;
        }
        if ((uint32_t)(now_ms - sensor->last_read_ms) > POLICY_TOF_STALE_MS) {
            out.stale_mask |= bit;
            fault_reasons |= POLICY_R_TOF_STALE;
            all_clear = 0U;
        }
        if (sensor->status == 0U && sensor->mm <= POLICY_STOP_MM) {
            out.obstacle_mask |= bit;
            fault_reasons |= POLICY_R_OBSTACLE;
        }
        if (sensor->mm < POLICY_CLEAR_MM)
            all_clear = 0U;
    }

    if (fault_reasons != 0U) {
        /* Packets received while blocked never acknowledge a future re-arm. */
        state->mode = POLICY_LATCHED;
        state->clear_timer_active = 0U;
        state->neutral_seen_during_clear = 0U;
        out.reasons = fault_reasons;
    } else if (state->mode == POLICY_LATCHED) {
        if (!all_clear) {
            state->clear_timer_active = 0U;
            state->neutral_seen_during_clear = 0U;
            out.reasons |= POLICY_R_WAIT_CLEAR;
        } else {
            if (!state->clear_timer_active) {
                state->clear_since_ms = now_ms;
                state->clear_timer_active = 1U;
            }
            if (new_command && fresh_command)
                state->neutral_seen_during_clear = (uint8_t)(cmd->esc == POLICY_ESC_MID);
            out.clear_elapsed_ms = (uint32_t)(now_ms - state->clear_since_ms);
            if (out.clear_elapsed_ms < POLICY_CLEAR_HOLD_MS)
                out.reasons |= POLICY_R_WAIT_CLEAR;
            if (!state->neutral_seen_during_clear || cmd->esc != POLICY_ESC_MID)
                out.reasons |= POLICY_R_WAIT_NEUTRAL;
            if (out.reasons == 0U) {
                /* This call ALWAYS outputs neutral. A later new generation is
                 * required before throttle is accepted. */
                state->mode = POLICY_ARMED_NEUTRAL;
                out.reasons = POLICY_R_WAIT_NEW_THROTTLE;
            }
        }
    } else if (state->mode == POLICY_ARMED_NEUTRAL) {
        if (new_command && cmd->esc != POLICY_ESC_MID) {
            state->mode = POLICY_RUNNING;
            out.esc = cmd->esc;
        } else {
            out.reasons = POLICY_R_WAIT_NEW_THROTTLE;
        }
    } else {
        out.esc = cmd->esc;
        if (cmd->esc == POLICY_ESC_MID) {
            state->mode = POLICY_ARMED_NEUTRAL;
            out.reasons = POLICY_R_WAIT_NEW_THROTTLE;
        }
    }

    out.mode = state->mode;
    out.neutral_seen_during_clear = state->neutral_seen_during_clear;
    return out;
}
