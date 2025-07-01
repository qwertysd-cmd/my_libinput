/*
 * Copyright © 2014-2015 Red Hat, Inc.
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

#include "evdev-mt-touchpad-tds.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "inttypes.h"
#include "evdev.h" /* For evdev_device_mm_to_units, evdev_log_* */
#include "filter.h" /* For filter_dispatch */
#include "libinput-private.h" /* For pointer_notify_motion, libinput_now */

static FILE *drag_log_file = NULL;

/* Enhanced FSM states */
enum edge_motion_state {
    STATE_IDLE,                    /* No tap-and-drag active */
    STATE_DRAG_ACTIVE_CENTERED,    /* Dragging, finger in center area */
    STATE_DRAG_EDGE_ENTRY,         /* Just entered edge while dragging */
    STATE_DRAG_EDGE_CONTINUOUS,    /* Continuously moving along edge */
    STATE_DRAG_EDGE_EXIT           /* Just exited edge, returning to center */
};

/* FSM context structure */
struct edge_motion_fsm {
    enum edge_motion_state current_state;
    enum edge_motion_state previous_state;
    uint64_t state_entry_time;      /* When we entered current state */
    uint64_t last_motion_time;      /* Last time we sent motion */
    uint32_t current_edge;          /* Current edge flags */
    uint32_t previous_edge;         /* Previous edge flags */
    double motion_dx;               /* Current motion direction X */
    double motion_dy;               /* Current motion direction Y */
    bool is_dragging;               /* Current drag state */
    bool was_dragging;              /* Previous drag state */
    uint64_t continuous_motion_count; /* Counter for continuous motions */
};

static struct edge_motion_fsm fsm = {
    .current_state = STATE_IDLE,
    .previous_state = STATE_IDLE,
    .state_entry_time = 0,
    .last_motion_time = 0,
    .current_edge = EDGE_NONE,
    .previous_edge = EDGE_NONE,
    .motion_dx = 0.0,
    .motion_dy = 0.0,
    .is_dragging = false,
    .was_dragging = false,
    .continuous_motion_count = 0
};

/* Configuration constants */
#define EDGE_MOTION_CONFIG_SPEED_MM_S 2.0          /* Motion speed in mm/s */
#define EDGE_MOTION_CONFIG_MIN_INTERVAL_US 8000     /* Min 8ms between motions */
#define EDGE_MOTION_CONFIG_EDGE_THRESHOLD_MM 7.0    /* 7mm edge detection threshold */

/* Convert FSM state to string for logging */
static const char *
state_to_string(enum edge_motion_state state)
{
    switch (state) {
    case STATE_IDLE:
        return "IDLE";
    case STATE_DRAG_ACTIVE_CENTERED:
        return "DRAG_ACTIVE_CENTERED";
    case STATE_DRAG_EDGE_ENTRY:
        return "DRAG_EDGE_ENTRY";
    case STATE_DRAG_EDGE_CONTINUOUS:
        return "DRAG_EDGE_CONTINUOUS";
    case STATE_DRAG_EDGE_EXIT:
        return "DRAG_EDGE_EXIT";
    default:
        return "UNKNOWN";
    }
}

/* Convert edge flags to string */
static const char *
edge_to_string(uint32_t edge)
{
    static char buffer[64];
    buffer[0] = '\0';

    if (edge == EDGE_NONE) {
        return "NONE";
    }

    if (edge & EDGE_LEFT) strcat(buffer, "LEFT|");
    if (edge & EDGE_RIGHT) strcat(buffer, "RIGHT|");
    if (edge & EDGE_TOP) strcat(buffer, "TOP|");
    if (edge & EDGE_BOTTOM) strcat(buffer, "BOTTOM|");

    /* Remove trailing | */
    size_t len = strlen(buffer);
    if (len > 0 && buffer[len-1] == '|') {
        buffer[len-1] = '\0';
    }

    return buffer;
}

/* Enhanced logging function */
static void
log_fsm_detailed(const char *event, uint64_t time, const char *details)
{
    if (!drag_log_file) return;

    fprintf(drag_log_file, "[%lu] %s: %s->%s | drag=%s->%s | edge=%s->%s | motion=(%+.2f,%+.2f) | count=%lu | %s\n",
            (unsigned long)(time / 1000), /* Convert to ms for readability */
            event,
            state_to_string(fsm.previous_state),
            state_to_string(fsm.current_state),
            fsm.was_dragging ? "T" : "F",
            fsm.is_dragging ? "T" : "F",
            edge_to_string(fsm.previous_edge),
            edge_to_string(fsm.current_edge),
            fsm.motion_dx,
            fsm.motion_dy,
            (unsigned long)fsm.continuous_motion_count,
            details ? details : "");
    fflush(drag_log_file);
}

/* Calculate motion direction based on edge */
static void
calculate_motion_vector(uint32_t edge, double *dx, double *dy)
{
    *dx = 0.0;
    *dy = 0.0;

    if (edge & EDGE_LEFT)
        *dx = -1.0;
    else if (edge & EDGE_RIGHT)
        *dx = 1.0;

    if (edge & EDGE_TOP)
        *dy = -1.0;
    else if (edge & EDGE_BOTTOM)
        *dy = 1.0;

    /* Normalize diagonal motion */
    double magnitude = sqrt((*dx) * (*dx) + (*dy) * (*dy));
    if (magnitude > 0) {
        *dx /= magnitude;
        *dy /= magnitude;
    }
}

/* Inject pointer motion with proper timing */
static void
inject_continuous_motion(struct tp_dispatch *tp, uint64_t time)
{
    struct device_float_coords raw;
    struct normalized_coords delta;
    uint64_t time_since_last = time - fsm.last_motion_time;

    /* Skip if too soon since last motion */
    if (fsm.last_motion_time != 0 && time_since_last < EDGE_MOTION_CONFIG_MIN_INTERVAL_US) {
        return;
    }

    /* Calculate distance based on actual time elapsed */
    double time_factor = (double)time_since_last / 1000000.0; /* Convert to seconds */
    double distance_mm = EDGE_MOTION_CONFIG_SPEED_MM_S * time_factor;

    /* Convert to device coordinates */
    raw.x = fsm.motion_dx * distance_mm * tp->accel.x_scale_coeff;
    raw.y = fsm.motion_dy * distance_mm * tp->accel.y_scale_coeff;

    /* Apply pointer acceleration filter */
    delta = filter_dispatch(tp->device->pointer.filter, &raw, tp, time);

    /* Post pointer motion event */
    pointer_notify_motion(&tp->device->base, time, &delta, &raw);

    /* Update timing and counter */
    fsm.last_motion_time = time;
    fsm.continuous_motion_count++;

    /* Log motion details */
    char details[128];
    snprintf(details, sizeof(details), "raw=(%.3f,%.3f) delta=(%.3f,%.3f) time_delta=%lums dist=%.2fmm",
             raw.x, raw.y, delta.x, delta.y,
             (unsigned long)(time_since_last / 1000), distance_mm);
    log_fsm_detailed("MOTION_INJECT", time, details);
}

/* Enhanced edge detection */
static uint32_t
detect_touch_edge(const struct tp_dispatch *tp, const struct tp_touch *t)
{
    uint32_t edge = EDGE_NONE;
    struct phys_coords mm = { EDGE_MOTION_CONFIG_EDGE_THRESHOLD_MM, EDGE_MOTION_CONFIG_EDGE_THRESHOLD_MM };
    struct device_coords edge_threshold;

    edge_threshold = evdev_device_mm_to_units(tp->device, &mm);

    /* Check each edge */
    if (tp->scroll.left_edge == 0) {
        if (t->point.x < edge_threshold.x)
            edge |= EDGE_LEFT;
    } else {
        if (t->point.x < tp->scroll.left_edge)
            edge |= EDGE_LEFT;
    }

    if (tp->scroll.upper_edge == 0) {
        if (t->point.y < edge_threshold.y)
            edge |= EDGE_TOP;
    } else {
        if (t->point.y < tp->scroll.upper_edge)
            edge |= EDGE_TOP;
    }

    if (tp->scroll.right_edge == 0) {
        int32_t max_x = tp->device->abs.absinfo_x->maximum;
        if (t->point.x > max_x - edge_threshold.x)
            edge |= EDGE_RIGHT;
    } else {
        if (t->point.x > tp->scroll.right_edge)
            edge |= EDGE_RIGHT;
    }

    if (tp->scroll.bottom_edge == 0) {
        int32_t max_y = tp->device->abs.absinfo_y->maximum;
        if (t->point.y > max_y - edge_threshold.y)
            edge |= EDGE_BOTTOM;
    } else {
        if (t->point.y > tp->scroll.bottom_edge)
            edge |= EDGE_BOTTOM;
    }

    return edge;
}

/* FSM state transition logic */
static enum edge_motion_state
calculate_next_state(bool is_dragging, uint32_t current_edge, enum edge_motion_state current_state)
{
    if (!is_dragging) {
        return STATE_IDLE;
    }

    switch (current_state) {
    case STATE_IDLE:
        if (current_edge != EDGE_NONE)
            return STATE_DRAG_EDGE_ENTRY;
        else
            return STATE_DRAG_ACTIVE_CENTERED;

    case STATE_DRAG_ACTIVE_CENTERED:
        if (current_edge != EDGE_NONE)
            return STATE_DRAG_EDGE_ENTRY;
        else
            return STATE_DRAG_ACTIVE_CENTERED;

    case STATE_DRAG_EDGE_ENTRY:
        if (current_edge != EDGE_NONE)
            return STATE_DRAG_EDGE_CONTINUOUS;
        else
            return STATE_DRAG_EDGE_EXIT;

    case STATE_DRAG_EDGE_CONTINUOUS:
        if (current_edge != EDGE_NONE)
            return STATE_DRAG_EDGE_CONTINUOUS;
        else
            return STATE_DRAG_EDGE_EXIT;

    case STATE_DRAG_EDGE_EXIT:
        if (current_edge != EDGE_NONE)
            return STATE_DRAG_EDGE_ENTRY;
        else
            return STATE_DRAG_ACTIVE_CENTERED;
    }

    return STATE_IDLE; /* Fallback */
}

/* Main FSM handler */
int
tp_edge_motion_handle_drag_state(struct tp_dispatch *tp, uint64_t time)
{
    struct tp_touch *t;
    uint32_t detected_edge = EDGE_NONE;
    bool drag_active = false;
    enum edge_motion_state next_state;

    /* Detect current drag state */
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
        break;
    }

    /* Find first active touch for edge detection */
    tp_for_each_touch(tp, t) {
        if (t->state == TOUCH_NONE || t->state == TOUCH_HOVERING || t->palm.state != PALM_NONE)
            continue;
        detected_edge = detect_touch_edge(tp, t);
        break;
    }

    /* Open log file if needed */
    if (!drag_log_file) {
        drag_log_file = fopen("/tmp/libinput-tap-drag-enhanced.log", "a");
        if (!drag_log_file) {
            evdev_log_error(tp->device, "Failed to open enhanced log file\n");
            return drag_active ? 1 : 0;
        }
        fprintf(drag_log_file, "\n=== NEW SESSION ===\n");
        fflush(drag_log_file);
    }

    /* Store previous state for comparison */
    fsm.previous_state = fsm.current_state;
    fsm.previous_edge = fsm.current_edge;
    fsm.was_dragging = fsm.is_dragging;

    /* Update current state */
    fsm.current_edge = detected_edge;
    fsm.is_dragging = drag_active;

    /* Calculate next FSM state */
    next_state = calculate_next_state(drag_active, detected_edge, fsm.current_state);

    /* Handle state transitions */
    if (next_state != fsm.current_state) {
        fsm.current_state = next_state;
        fsm.state_entry_time = time;

        /* Reset motion counter on state change */
        if (fsm.current_state != STATE_DRAG_EDGE_CONTINUOUS) {
            fsm.continuous_motion_count = 0;
        }

        log_fsm_detailed("STATE_TRANSITION", time, "");
    }

    /* Execute state-specific actions */
    switch (fsm.current_state) {
    case STATE_IDLE:
        fsm.motion_dx = 0.0;
        fsm.motion_dy = 0.0;
        fsm.continuous_motion_count = 0;
        break;

    case STATE_DRAG_ACTIVE_CENTERED:
        fsm.motion_dx = 0.0;
        fsm.motion_dy = 0.0;
        break;

    case STATE_DRAG_EDGE_ENTRY:
        calculate_motion_vector(fsm.current_edge, &fsm.motion_dx, &fsm.motion_dy);
        fsm.continuous_motion_count = 0;
        fsm.last_motion_time = 0; /* Reset to ensure immediate first motion */
        log_fsm_detailed("EDGE_ENTRY", time, "Preparing for continuous motion");
        break;

    case STATE_DRAG_EDGE_CONTINUOUS:
        calculate_motion_vector(fsm.current_edge, &fsm.motion_dx, &fsm.motion_dy);
        inject_continuous_motion(tp, time);
        break;

    case STATE_DRAG_EDGE_EXIT:
        fsm.motion_dx = 0.0;
        fsm.motion_dy = 0.0;
        log_fsm_detailed("EDGE_EXIT", time, "Stopping continuous motion");
        break;
    }

    /* Periodic detailed logging */
    static uint64_t last_detailed_log = 0;
    if (time - last_detailed_log > 100000) { /* Every 100ms */
        char details[256];
        snprintf(details, sizeof(details),
                 "state_duration=%lums tap_state=%d touch_coords=(%d,%d)",
                 (unsigned long)((time - fsm.state_entry_time) / 1000),
                 tp->tap.state,
                 t ? t->point.x : -1,
                 t ? t->point.y : -1);
        log_fsm_detailed("PERIODIC_STATUS", time, details);
        last_detailed_log = time;
    }

    return drag_active ? 1 : 0;
}

/* Cleanup function */
void
tp_edge_motion_cleanup(void)
{
    if (drag_log_file) {
        log_fsm_detailed("CLEANUP", 0, "Shutting down FSM");
        fclose(drag_log_file);
        drag_log_file = NULL;
    }

    /* Reset FSM to initial state */
    memset(&fsm, 0, sizeof(fsm));
    fsm.current_state = STATE_IDLE;
    fsm.previous_state = STATE_IDLE;
}
