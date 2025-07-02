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


/*
 * TOUCHPAD EDGE MOTION
 * ====================
 *
 * This module implements cursor motion when you are performing "tap-and-drag"
 * and reach the edges of a touchpad. the implementation automatically continues
 * moving the cursor in that direction to allow selection/dragging of content
 * that extends beyond the physical touchpad boundaries.
 *
 * The speed is now dynamic based on distance from edge:
 * - 5-7mm: 0.5x speed multiplier
 * - 3-5mm: 1.0x speed multiplier
 * - 0-3mm: 2.0x speed multiplier
 * Separate multipliers are applied for X and Y axes and updated dynamically
 * as the finger moves closer to or further from each edge.
 */

#include "evdev-mt-touchpad-tds.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "inttypes.h"
#include "evdev.h"
#include "filter.h"
#include "libinput-private.h"

void tp_edge_motion_init(struct tp_dispatch *tp);
struct tp_dispatch;

/*
static FILE *drag_log_file = NULL;
*/

enum edge_motion_state {
    STATE_IDLE,
    STATE_DRAG_ACTIVE_CENTERED,
    STATE_DRAG_EDGE_ENTRY,
    STATE_DRAG_EDGE_CONTINUOUS,
    STATE_DRAG_EDGE_EXIT
};

struct edge_motion_fsm {
    enum edge_motion_state current_state;
    enum edge_motion_state previous_state;
    uint64_t state_entry_time;
    uint64_t last_motion_time;
    uint32_t current_edge;
    uint32_t previous_edge;
    double motion_dx;
    double motion_dy;
    double speed_multiplier_x;  /* Dynamic speed multiplier for X axis */
    double speed_multiplier_y;  /* Dynamic speed multiplier for Y axis */
    bool is_dragging;
    bool was_dragging;
    uint64_t continuous_motion_count;
    struct tp_dispatch *tp;
    struct tp_touch *active_touch; /* Track the active touch for dynamic updates */
    struct libinput_timer timer;
};

static struct edge_motion_fsm fsm = {
    .current_state = STATE_IDLE,
    .previous_state = STATE_IDLE,
    .tp = NULL,
    .active_touch = NULL,
};

#define EDGE_MOTION_CONFIG_SPEED_MM_S 40.0
#define EDGE_MOTION_CONFIG_MIN_INTERVAL_US 8000
#define EDGE_MOTION_CONFIG_EDGE_THRESHOLD_MM 7.0

/* Dynamic speed zone thresholds */
#define EDGE_ZONE_FAR_MM 7.0      /* 5-7mm zone */
#define EDGE_ZONE_MID_MM 5.0      /* 3-5mm zone */
#define EDGE_ZONE_NEAR_MM 3.0     /* 0-3mm zone */

/* Speed multipliers for each zone */
#define SPEED_MULT_FAR 0.5        /* 5-7mm */
#define SPEED_MULT_MID 1.0        /* 3-5mm */
#define SPEED_MULT_NEAR 2.0       /* 0-3mm */

/*
static const char *
state_to_string(enum edge_motion_state state) {
    switch (state) {
        case STATE_IDLE: return "IDLE";
        case STATE_DRAG_ACTIVE_CENTERED: return "DRAG_ACTIVE_CENTERED";
        case STATE_DRAG_EDGE_ENTRY: return "DRAG_EDGE_ENTRY";
        case STATE_DRAG_EDGE_CONTINUOUS: return "DRAG_EDGE_CONTINUOUS";
        case STATE_DRAG_EDGE_EXIT: return "DRAG_EDGE_EXIT";
        default: return "UNKNOWN";
    }
}

static const char *
edge_to_string(uint32_t edge) {
    static char buffer[64];
    buffer[0] = '\0';
    if (edge == EDGE_NONE) return "NONE";
    if (edge & EDGE_LEFT) strcat(buffer, "LEFT|");
    if (edge & EDGE_RIGHT) strcat(buffer, "RIGHT|");
    if (edge & EDGE_TOP) strcat(buffer, "TOP|");
    if (edge & EDGE_BOTTOM) strcat(buffer, "BOTTOM|");
    size_t len = strlen(buffer);
    if (len > 0) buffer[len - 1] = '\0';
    return buffer;
}
*/

/*
static void
log_fsm_detailed(const char *event, uint64_t time, const char *details) {
    if (!drag_log_file) return;
    fprintf(drag_log_file, "[%lu] %s: %s->%s | drag=%s->%s | edge=%s->%s | motion=(%+.2f,%+.2f) | speed_mult=(%.2f,%.2f) | count=%lu | %s\n",
            (unsigned long)(time / 1000), event, state_to_string(fsm.previous_state),
            state_to_string(fsm.current_state), fsm.was_dragging ? "T" : "F", fsm.is_dragging ? "T" : "F",
            edge_to_string(fsm.previous_edge), edge_to_string(fsm.current_edge), fsm.motion_dx, fsm.motion_dy,
            fsm.speed_multiplier_x, fsm.speed_multiplier_y,
            (unsigned long)fsm.continuous_motion_count, details ? details : "");
    fflush(drag_log_file);
}
*/

static double
get_speed_multiplier_for_distance(double distance_mm) {
    if (distance_mm >= EDGE_ZONE_MID_MM) {
        return SPEED_MULT_FAR;   /* 5-7mm zone */
    } else if (distance_mm >= EDGE_ZONE_NEAR_MM) {
        return SPEED_MULT_MID;   /* 2-5mm zone */
    } else {
        return SPEED_MULT_NEAR;  /* 0-2mm zone */
    }
}

static void
update_motion_vector_and_speed(const struct tp_dispatch *tp, const struct tp_touch *t,
                               uint32_t edge) {
    fsm.motion_dx = 0.0;
    fsm.motion_dy = 0.0;
    fsm.speed_multiplier_x = 1.0;
    fsm.speed_multiplier_y = 1.0;

    if (!t) return; /* Safety check */

    /* Convert touch coordinates to millimeters for distance calculation */
    struct device_coords touch_pos = {t->point.x, t->point.y};
    struct phys_coords touch_mm = evdev_device_units_to_mm(tp->device, &touch_pos);

    /* Get touchpad dimensions in mm */
    struct device_coords max_coords = {
        tp->device->abs.absinfo_x->maximum,
        tp->device->abs.absinfo_y->maximum
    };
    struct phys_coords max_mm = evdev_device_units_to_mm(tp->device, &max_coords);

    /* Calculate distances from each edge in mm */
    double dist_left = touch_mm.x;
    double dist_right = max_mm.x - touch_mm.x;
    double dist_top = touch_mm.y;
    double dist_bottom = max_mm.y - touch_mm.y;

    /* Determine motion direction and calculate speed multipliers */
    if (edge & EDGE_LEFT) {
        fsm.motion_dx = -1.0;
        fsm.speed_multiplier_x = get_speed_multiplier_for_distance(dist_left);
    } else if (edge & EDGE_RIGHT) {
        fsm.motion_dx = 1.0;
        fsm.speed_multiplier_x = get_speed_multiplier_for_distance(dist_right);
    }

    if (edge & EDGE_TOP) {
        fsm.motion_dy = -1.0;
        fsm.speed_multiplier_y = get_speed_multiplier_for_distance(dist_top);
    } else if (edge & EDGE_BOTTOM) {
        fsm.motion_dy = 1.0;
        fsm.speed_multiplier_y = get_speed_multiplier_for_distance(dist_bottom);
    }

    /* Normalize diagonal motion */
    double mag = sqrt((fsm.motion_dx) * (fsm.motion_dx) + (fsm.motion_dy) * (fsm.motion_dy));
    if (mag > 0) {
        fsm.motion_dx /= mag;
        fsm.motion_dy /= mag;
    }
}

static void
inject_accumulated_motion(struct tp_dispatch *tp, uint64_t time) {
    /* Initialize timing on first call */
    if (fsm.last_motion_time == 0) {
        fsm.last_motion_time = time;
        return;
    }

    /* Calculate time delta since last motion event */
    uint64_t time_since_last = time - fsm.last_motion_time;

    /* Convert time delta to base distance based on configured speed */
    /* time_since_last is in microseconds, speed is mm/s */
    double base_dist_mm = EDGE_MOTION_CONFIG_SPEED_MM_S * ((double)time_since_last / 1000000.0);

    /* Skip micro-movements to avoid jitter */
    if (base_dist_mm < 0.001) return;

    /* Update motion vector and speed multipliers dynamically before each motion event */
    /* This ensures speed changes immediately as finger moves closer/further from edges */
    if (fsm.active_touch && fsm.current_edge != EDGE_NONE) {
        update_motion_vector_and_speed(fsm.tp, fsm.active_touch, fsm.current_edge);
    }

    /* Apply dynamic speed multipliers separately for X and Y */
    double actual_dist_x = base_dist_mm * fsm.speed_multiplier_x;
    double actual_dist_y = base_dist_mm * fsm.speed_multiplier_y;

    /* Create raw motion coordinates in device units */
    struct device_float_coords raw = {
        .x = fsm.motion_dx * actual_dist_x * tp->accel.x_scale_coeff,
        .y = fsm.motion_dy * actual_dist_y * tp->accel.y_scale_coeff
    };

    /* Apply pointer acceleration and user preferences */
    struct normalized_coords delta = filter_dispatch(tp->device->pointer.filter, &raw, tp, time);

    /* Send motion event to compositor */
    pointer_notify_motion(&tp->device->base, time, &delta, &raw);

    /* Update timing and statistics */
    fsm.last_motion_time = time;
    fsm.continuous_motion_count++;
}

static uint32_t
detect_touch_edge(const struct tp_dispatch *tp, const struct tp_touch *t) {
    uint32_t edge = EDGE_NONE;

    /* Convert threshold from millimeters to device units */
    struct phys_coords mm = {EDGE_MOTION_CONFIG_EDGE_THRESHOLD_MM, EDGE_MOTION_CONFIG_EDGE_THRESHOLD_MM};
    struct device_coords threshold = evdev_device_mm_to_units(tp->device, &mm);

    /* Check each edge boundary */
    if (t->point.x < threshold.x)
        edge |= EDGE_LEFT;
    if (t->point.x > tp->device->abs.absinfo_x->maximum - threshold.x)
        edge |= EDGE_RIGHT;
    if (t->point.y < threshold.y)
        edge |= EDGE_TOP;
    if (t->point.y > tp->device->abs.absinfo_y->maximum - threshold.y)
        edge |= EDGE_BOTTOM;

    return edge;
}

static enum edge_motion_state
calculate_next_state(bool is_dragging, uint32_t edge, enum edge_motion_state current) {
    /* If not dragging, always return to idle state */
    if (!is_dragging) return STATE_IDLE;

    switch (current) {
        /* From stable non-edge states, transition based on edge contact */
        case STATE_IDLE:
        case STATE_DRAG_ACTIVE_CENTERED:
        case STATE_DRAG_EDGE_EXIT:
            return (edge != EDGE_NONE) ? STATE_DRAG_EDGE_ENTRY : STATE_DRAG_ACTIVE_CENTERED;

        /* From edge-active states, maintain edge state or exit */
        case STATE_DRAG_EDGE_ENTRY:
        case STATE_DRAG_EDGE_CONTINUOUS:
            return (edge != EDGE_NONE) ? STATE_DRAG_EDGE_CONTINUOUS : STATE_DRAG_EDGE_EXIT;
    }

    /* Defensive fallback for any unhandled states */
    return STATE_IDLE;
}

static void
tp_edge_motion_handle_timeout(uint64_t now, void *data) {
    struct edge_motion_fsm *fsm_ptr = data;

    if (fsm_ptr->current_state != STATE_DRAG_EDGE_ENTRY &&
        fsm_ptr->current_state != STATE_DRAG_EDGE_CONTINUOUS) {
        return;  /* Timer should have been cancelled, but be defensive */
    }

    /* Generate motion event based on current motion parameters */
    /* The inject_accumulated_motion function will update speed multipliers dynamically */
    inject_accumulated_motion(fsm_ptr->tp, now);

    /* Schedule next motion event */
    libinput_timer_set(&fsm_ptr->timer, now + EDGE_MOTION_CONFIG_MIN_INTERVAL_US);
}

void
tp_edge_motion_init(struct tp_dispatch *tp) {
    if (fsm.tp) return;
    memset(&fsm, 0, sizeof(fsm));
    fsm.current_state = STATE_IDLE;
    fsm.tp = tp;
    fsm.active_touch = NULL;
    libinput_timer_init(&fsm.timer, tp_libinput_context(tp), "edge drag motion",
                        tp_edge_motion_handle_timeout, &fsm);
}

void
tp_edge_motion_cleanup(void) {

    /*
    if (drag_log_file) {
        log_fsm_detailed("CLEANUP", 0, "Shutting down FSM");
        fclose(drag_log_file);
        drag_log_file = NULL;
    }
    */

    /* Clean up timer resources if the FSM was initialized with a touchpad */
    if (fsm.tp) {
        libinput_timer_destroy(&fsm.timer);
    }

    /* Reset the entire FSM structure to zero */
    memset(&fsm, 0, sizeof(fsm));

    fsm.current_state = STATE_IDLE;
    fsm.active_touch = NULL;
}

int
tp_edge_motion_handle_drag_state(struct tp_dispatch *tp, uint64_t time) {
    /* Initialize the FSM if this is the first call */
    if (!fsm.tp)
        tp_edge_motion_init(tp);

    /*
     * Determine if a drag operation is currently active by checking the tap FSM state
     */
    bool drag_active = false;
    switch (tp->tap.state) {
        case TAP_STATE_1FGTAP_DRAGGING:
        case TAP_STATE_1FGTAP_DRAGGING_2:
        case TAP_STATE_1FGTAP_DRAGGING_WAIT:
        case TAP_STATE_1FGTAP_DRAGGING_OR_TAP:
        case TAP_STATE_1FGTAP_DRAGGING_OR_DOUBLETAP:
            drag_active = true;
            break;
        default:
            drag_active = false;
    }

    /*
     * Find active touch and check for edge detection when drag is active
     */
    uint32_t detected_edge = EDGE_NONE;
    struct tp_touch *active_touch = NULL;
    if (drag_active) {
        /* Iterate through all active touches to find edge contact */
        struct tp_touch *t;
        tp_for_each_touch(tp, t) {
            /* Only consider touches that are actually in contact with the surface */
            if (t->state != TOUCH_NONE && t->state != TOUCH_HOVERING) {
                detected_edge = detect_touch_edge(tp, t);
                active_touch = t;
                break; /* Only need to find the first active touch at an edge */
            }
        }
    }

    /* Store the active touch for dynamic updates during timer callbacks */
    fsm.active_touch = active_touch;

    /*
    if (!drag_log_file) {
        drag_log_file = fopen("/tmp/libinput-tap-drag-enhanced.log", "a");
        if(drag_log_file) {
            fprintf(drag_log_file, "\n=== NEW SESSION ===\n");
            fflush(drag_log_file);
        }
    }
    */

    /* Update FSM state variables with current conditions */
    fsm.previous_state = fsm.current_state;
    fsm.current_edge = detected_edge;
    fsm.is_dragging = drag_active;

    /* Calculate the next state based on current drag status and edge detection */
    enum edge_motion_state next_state = calculate_next_state(drag_active, detected_edge, fsm.current_state);

    /* Handle state transitions */
    if (next_state != fsm.current_state) {
        fsm.current_state = next_state;
        fsm.state_entry_time = time;

        /* Reset continuous motion counter when leaving continuous motion state */
        if (fsm.current_state != STATE_DRAG_EDGE_CONTINUOUS)
            fsm.continuous_motion_count = 0;

        //log_fsm_detailed("STATE_TRANSITION", time, "");
    }

    /* Handle state-specific actions */
    switch (fsm.current_state) {
        case STATE_IDLE:
            /* No drag active - cancel any pending timers and clear active touch */
            libinput_timer_cancel(&fsm.timer);
            fsm.active_touch = NULL;
            break;

        case STATE_DRAG_ACTIVE_CENTERED:
            /* Drag is active but not at edge - stop generated motion and clear active touch */
            libinput_timer_cancel(&fsm.timer);
            fsm.active_touch = NULL;
            break;

        case STATE_DRAG_EDGE_EXIT:
            /* Touch has moved away from edge - stop generated motion and clear active touch */
            libinput_timer_cancel(&fsm.timer);
            fsm.active_touch = NULL;
            break;

        case STATE_DRAG_EDGE_ENTRY:
            /* Touch has just reached an edge - start generated motion */
            if (active_touch) {
                update_motion_vector_and_speed(tp, active_touch, fsm.current_edge);
            }
            fsm.last_motion_time = time;
            tp_edge_motion_handle_timeout(time, &fsm); /* Start the timer-based motion loop */
            break;

        case STATE_DRAG_EDGE_CONTINUOUS:
            /* Continuing motion at edge - motion vector and speed will be updated dynamically in timer */
            /* No action needed here as updates happen in inject_accumulated_motion() */
            break;
    }

    /*
     * Return whether generated motion should be active
     * Returns 1 (true) when in edge motion states, 0 (false) for idle or centered states
     * FIXED: Use correct logical operator and state checks like the original working code
     */
    return (fsm.current_state != STATE_DRAG_ACTIVE_CENTERED && fsm.current_state != STATE_IDLE);
}
