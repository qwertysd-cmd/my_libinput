/*
 * Copyright © 2014-2015 QWERTYSD-CMD.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "evdev-mt-touchpad-4finger.h"
#include "evdev-mt-touchpad.h"
#include "evdev.h"
#include <linux/input-event-codes.h>
#include <math.h>

/* FSM States */
enum swipe_state {
    SWIPE_STATE_IDLE,
    SWIPE_STATE_DETECTING,
    SWIPE_STATE_VERTICAL_ACTIVE,
    SWIPE_STATE_HORIZONTAL_ACTIVE,
    SWIPE_STATE_COOLDOWN
};

/* Swipe directions */
enum swipe_direction {
    SWIPE_DIR_NONE,
    SWIPE_DIR_UP,
    SWIPE_DIR_DOWN,
    SWIPE_DIR_LEFT,
    SWIPE_DIR_RIGHT
};

/* FSM Configuration */
#define SWIPE_DETECTION_THRESHOLD 3        /* Consecutive movements needed to lock direction */
#define SWIPE_MIN_DELTA_THRESHOLD 0.15     /* Minimum delta to count as movement */
#define SWIPE_AXIS_LOCK_RATIO 1.5          /* Ratio to determine primary axis */
#define SWIPE_BASE_INTERVAL_MS 80          /* Base interval between key presses (80ms) */
#define SWIPE_COOLDOWN_MS 150              /* Cooldown after gesture ends */
#define SWIPE_INACTIVITY_TIMEOUT_MS 200    /* Reset if no movement for this long */

/* Speed scaling factors */
#define SWIPE_SPEED_SCALE_SLOW 2.0         /* Multiplier for slow movements */
#define SWIPE_SPEED_SCALE_FAST 0.5         /* Multiplier for fast movements */
#define SWIPE_FAST_THRESHOLD 15.0          /* Delta threshold for "fast" movement */
#define SWIPE_SLOW_THRESHOLD 2.0           /* Delta threshold for "slow" movement */

/* Accumulation thresholds for key generation */
#define SWIPE_VOLUME_THRESHOLD 8.0         /* Accumulated delta needed for volume change */
#define SWIPE_BRIGHTNESS_THRESHOLD 10.0    /* Accumulated delta needed for brightness change */

struct swipe_fsm {
    enum swipe_state state;
    enum swipe_direction locked_direction;
    uint64_t last_event_time;
    uint64_t last_key_time;
    uint64_t state_enter_time;

    /* Direction detection */
    int consecutive_count;
    enum swipe_direction candidate_direction;

    /* Movement accumulation */
    double accumulated_delta;
    double total_movement;
    uint32_t movement_count;

    /* Timer for state management */
    struct libinput_timer timer;
    struct tp_dispatch *tp;

    /* Statistics */
    uint32_t keys_sent;
    bool timer_active;
};

static struct swipe_fsm fsm = {
    .state = SWIPE_STATE_IDLE,
    .locked_direction = SWIPE_DIR_NONE,
    .tp = NULL,
    .timer_active = false
};

static const char *
state_to_string(enum swipe_state state) {
    switch (state) {
        case SWIPE_STATE_IDLE: return "IDLE";
        case SWIPE_STATE_DETECTING: return "DETECTING";
        case SWIPE_STATE_VERTICAL_ACTIVE: return "VERTICAL_ACTIVE";
        case SWIPE_STATE_HORIZONTAL_ACTIVE: return "HORIZONTAL_ACTIVE";
        case SWIPE_STATE_COOLDOWN: return "COOLDOWN";
        default: return "UNKNOWN";
    }
}

static const char *
direction_to_string(enum swipe_direction dir) {
    switch (dir) {
        case SWIPE_DIR_NONE: return "NONE";
        case SWIPE_DIR_UP: return "UP";
        case SWIPE_DIR_DOWN: return "DOWN";
        case SWIPE_DIR_LEFT: return "LEFT";
        case SWIPE_DIR_RIGHT: return "RIGHT";
        default: return "UNKNOWN";
    }
}

static enum swipe_direction
get_primary_direction(const struct normalized_coords *delta) {
    double abs_x = fabs(delta->x);
    double abs_y = fabs(delta->y);

    /* Check if movement is significant enough */
    if (abs_x < SWIPE_MIN_DELTA_THRESHOLD && abs_y < SWIPE_MIN_DELTA_THRESHOLD) {
        return SWIPE_DIR_NONE;
    }

    /* Determine primary axis with ratio check */
    if (abs_y > abs_x * SWIPE_AXIS_LOCK_RATIO) {
        return (delta->y < 0) ? SWIPE_DIR_UP : SWIPE_DIR_DOWN;
    } else if (abs_x > abs_y * SWIPE_AXIS_LOCK_RATIO) {
        return (delta->x < 0) ? SWIPE_DIR_LEFT : SWIPE_DIR_RIGHT;
    }

    return SWIPE_DIR_NONE; /* Movement is too ambiguous */
}

static double
calculate_speed_multiplier(double delta_magnitude) {
    if (delta_magnitude >= SWIPE_FAST_THRESHOLD) {
        return SWIPE_SPEED_SCALE_FAST;
    } else if (delta_magnitude <= SWIPE_SLOW_THRESHOLD) {
        return SWIPE_SPEED_SCALE_SLOW;
    }

    /* Linear interpolation between slow and fast */
    double ratio = (delta_magnitude - SWIPE_SLOW_THRESHOLD) /
                   (SWIPE_FAST_THRESHOLD - SWIPE_SLOW_THRESHOLD);
    return SWIPE_SPEED_SCALE_SLOW + ratio * (SWIPE_SPEED_SCALE_FAST - SWIPE_SPEED_SCALE_SLOW);
}

static void
send_key_event(struct tp_dispatch *tp, uint32_t keycode, uint64_t time) {
    if (keycode == 0) return;

    keycode_t key = keycode_from_uint32_t(keycode);
    keyboard_notify_key(&tp->device->base, time, key, LIBINPUT_KEY_STATE_PRESSED);
    keyboard_notify_key(&tp->device->base, time, key, LIBINPUT_KEY_STATE_RELEASED);

    fsm.keys_sent++;
    fsm.last_key_time = time;
}

static void
process_accumulated_movement(struct tp_dispatch *tp, uint64_t time) {
    double threshold;
    uint32_t keycode = 0;

    if (fsm.locked_direction == SWIPE_DIR_NONE) return;

    /* Determine threshold and keycode based on direction */
    switch (fsm.locked_direction) {
        case SWIPE_DIR_UP:
            threshold = SWIPE_VOLUME_THRESHOLD;
            keycode = KEY_VOLUMEUP;
            break;
        case SWIPE_DIR_DOWN:
            threshold = SWIPE_VOLUME_THRESHOLD;
            keycode = KEY_VOLUMEDOWN;
            break;
        case SWIPE_DIR_LEFT:
            threshold = SWIPE_BRIGHTNESS_THRESHOLD;
            keycode = KEY_BRIGHTNESSDOWN;
            break;
        case SWIPE_DIR_RIGHT:
            threshold = SWIPE_BRIGHTNESS_THRESHOLD;
            keycode = KEY_BRIGHTNESSUP;
            break;
        default:
            return;
    }

    /* Check if we've accumulated enough movement */
    if (fabs(fsm.accumulated_delta) >= threshold) {
        send_key_event(tp, keycode, time);

        /* Reset accumulation but keep the sign for continuous movement */
        fsm.accumulated_delta = fsm.accumulated_delta > 0 ?
            fsm.accumulated_delta - threshold :
            fsm.accumulated_delta + threshold;
    }
}

static void
swipe_timer_handler(uint64_t now, void *data) {
    struct swipe_fsm *fsm_ptr = data;

    /* Check for inactivity timeout */
    if (now - fsm_ptr->last_event_time > SWIPE_INACTIVITY_TIMEOUT_MS * 1000) {
        /* Reset to idle due to inactivity */
        fsm_ptr->state = SWIPE_STATE_IDLE;
        fsm_ptr->locked_direction = SWIPE_DIR_NONE;
        fsm_ptr->consecutive_count = 0;
        fsm_ptr->accumulated_delta = 0.0;
        fsm_ptr->timer_active = false;
        libinput_timer_cancel(&fsm_ptr->timer);
        return;
    }

    /* Handle cooldown expiration */
    if (fsm_ptr->state == SWIPE_STATE_COOLDOWN) {
        fsm_ptr->state = SWIPE_STATE_IDLE;
        fsm_ptr->locked_direction = SWIPE_DIR_NONE;
        fsm_ptr->consecutive_count = 0;
        fsm_ptr->accumulated_delta = 0.0;
        fsm_ptr->timer_active = false;
        return;
    }

    /* Process any accumulated movement */
    if (fsm_ptr->state == SWIPE_STATE_VERTICAL_ACTIVE ||
        fsm_ptr->state == SWIPE_STATE_HORIZONTAL_ACTIVE) {
        process_accumulated_movement(fsm_ptr->tp, now);
    }

    /* Schedule next timer if still active */
    if (fsm_ptr->timer_active) {
        libinput_timer_set(&fsm_ptr->timer, now + SWIPE_BASE_INTERVAL_MS * 1000);
    }
}

static void
swipe_fsm_init(struct tp_dispatch *tp) {
    if (fsm.tp) return; /* Already initialized */

    memset(&fsm, 0, sizeof(fsm));
    fsm.state = SWIPE_STATE_IDLE;
    fsm.locked_direction = SWIPE_DIR_NONE;
    fsm.tp = tp;

    libinput_timer_init(&fsm.timer, tp_libinput_context(tp), "4finger-swipe",
                        swipe_timer_handler, &fsm);
}

static void
start_timer_if_needed(uint64_t time) {
    if (!fsm.timer_active) {
        fsm.timer_active = true;
        libinput_timer_set(&fsm.timer, time + SWIPE_BASE_INTERVAL_MS * 1000);
    }
}

static void
log_swipe_event(const char *event_type, int finger_count, const struct normalized_coords *delta, uint32_t keycode) {
    FILE *log_file;
    char *log_path = "/tmp/touchpad_swipe.log";

    log_file = fopen(log_path, "a");
    if (log_file == NULL) {
        return;
    }

    fprintf(log_file, "%s: %d-finger %s | delta(%.2f,%.2f) | accum=%.2f | keys=%u | state=%s->%s\n",
            event_type, finger_count, direction_to_string(fsm.locked_direction),
            delta->x, delta->y, fsm.accumulated_delta, fsm.keys_sent,
            state_to_string(fsm.state), direction_to_string(fsm.locked_direction));

    fclose(log_file);
}

/* Main function called by the gesture recognition system */
void
tp_deal_with_it(struct tp_dispatch *tp, uint64_t time, const char *event_type, int finger_count, const struct normalized_coords *delta)
{
    /* Initialize FSM if needed */
    if (!fsm.tp) {
        swipe_fsm_init(tp);
    }

    /* Only process 4-finger swipes */
    if (finger_count != 4) {
        return;
    }

    /* Update timing */
    fsm.last_event_time = time;

    /* Get primary direction for this movement */
    enum swipe_direction current_direction = get_primary_direction(delta);

    /* Calculate movement magnitude */
    double delta_magnitude = sqrt(delta->x * delta->x + delta->y * delta->y);

    /* FSM State Machine */
    switch (fsm.state) {
        case SWIPE_STATE_IDLE:
            if (current_direction != SWIPE_DIR_NONE) {
                fsm.state = SWIPE_STATE_DETECTING;
                fsm.candidate_direction = current_direction;
                fsm.consecutive_count = 1;
                fsm.state_enter_time = time;
                start_timer_if_needed(time);
            }
            break;

        case SWIPE_STATE_DETECTING:
            if (current_direction == fsm.candidate_direction) {
                fsm.consecutive_count++;
                if (fsm.consecutive_count >= SWIPE_DETECTION_THRESHOLD) {
                    /* Lock in the direction */
                    fsm.locked_direction = fsm.candidate_direction;
                    fsm.accumulated_delta = 0.0;
                    fsm.total_movement = 0.0;
                    fsm.movement_count = 0;
                    fsm.keys_sent = 0;

                    /* Transition to appropriate active state */
                    if (fsm.locked_direction == SWIPE_DIR_UP || fsm.locked_direction == SWIPE_DIR_DOWN) {
                        fsm.state = SWIPE_STATE_VERTICAL_ACTIVE;
                    } else {
                        fsm.state = SWIPE_STATE_HORIZONTAL_ACTIVE;
                    }
                }
            } else if (current_direction != SWIPE_DIR_NONE) {
                /* Direction changed, reset detection */
                fsm.candidate_direction = current_direction;
                fsm.consecutive_count = 1;
            } else {
                /* No significant movement, maintain current detection */
            }
            break;

        case SWIPE_STATE_VERTICAL_ACTIVE:
            if (current_direction == SWIPE_DIR_UP || current_direction == SWIPE_DIR_DOWN ||
                current_direction == SWIPE_DIR_NONE) {
                /* Valid movement in locked axis or no movement */
                if (current_direction != SWIPE_DIR_NONE) {
                    /* Allow immediate direction switching within the same axis */
                    if (current_direction != fsm.locked_direction) {
                        fsm.locked_direction = current_direction;
                        /* Reset accumulation when switching directions */
                        fsm.accumulated_delta = 0.0;
                    }

                    double speed_mult = calculate_speed_multiplier(delta_magnitude);
                    double effective_delta = (fsm.locked_direction == SWIPE_DIR_UP) ? -delta->y : delta->y;

                    fsm.accumulated_delta += effective_delta * speed_mult;
                    fsm.total_movement += fabs(effective_delta);
                    fsm.movement_count++;

                    process_accumulated_movement(tp, time);
                }
            }
            /* Ignore cross-axis movements when locked */
            break;

        case SWIPE_STATE_HORIZONTAL_ACTIVE:
            if (current_direction == SWIPE_DIR_LEFT || current_direction == SWIPE_DIR_RIGHT ||
                current_direction == SWIPE_DIR_NONE) {
                /* Valid movement in locked axis or no movement */
                if (current_direction != SWIPE_DIR_NONE) {
                    /* Allow immediate direction switching within the same axis */
                    if (current_direction != fsm.locked_direction) {
                        fsm.locked_direction = current_direction;
                        /* Reset accumulation when switching directions */
                        fsm.accumulated_delta = 0.0;
                    }

                    double speed_mult = calculate_speed_multiplier(delta_magnitude);
                    double effective_delta = (fsm.locked_direction == SWIPE_DIR_RIGHT) ? delta->x : -delta->x;

                    fsm.accumulated_delta += effective_delta * speed_mult;
                    fsm.total_movement += fabs(effective_delta);
                    fsm.movement_count++;

                    process_accumulated_movement(tp, time);
                }
            }
            /* Ignore cross-axis movements when locked */
            break;

        case SWIPE_STATE_COOLDOWN:
            /* Ignore all input during cooldown */
            break;
    }

    /* Log the event for debugging */
    log_swipe_event(event_type, finger_count, delta,
                   (fsm.locked_direction == SWIPE_DIR_UP) ? KEY_VOLUMEUP :
                   (fsm.locked_direction == SWIPE_DIR_DOWN) ? KEY_VOLUMEDOWN :
                   (fsm.locked_direction == SWIPE_DIR_LEFT) ? KEY_BRIGHTNESSDOWN :
                   (fsm.locked_direction == SWIPE_DIR_RIGHT) ? KEY_BRIGHTNESSUP : 0);
}

/* Cleanup function */
void
tp_swipe_fsm_cleanup(void) {
    if (fsm.tp) {
        libinput_timer_cancel(&fsm.timer);
        libinput_timer_destroy(&fsm.timer);
    }

    memset(&fsm, 0, sizeof(fsm));
    fsm.state = SWIPE_STATE_IDLE;
}
