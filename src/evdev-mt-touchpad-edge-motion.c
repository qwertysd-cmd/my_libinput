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
#include "evdev.h"
#include "filter.h"
#include "libinput-private.h"

struct tp_dispatch;

static FILE *drag_log_file = NULL;

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
    bool is_dragging;
    bool was_dragging;
    uint64_t continuous_motion_count;
    struct tp_dispatch *tp;
    struct libinput_timer timer;
};

static struct edge_motion_fsm fsm = {
    .current_state = STATE_IDLE,
    .previous_state = STATE_IDLE,
    .tp = NULL,
};

#define EDGE_MOTION_CONFIG_SPEED_MM_S 25.0
#define EDGE_MOTION_CONFIG_MIN_INTERVAL_US 8000
#define EDGE_MOTION_CONFIG_EDGE_THRESHOLD_MM 7.0

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

static void
log_fsm_detailed(const char *event, uint64_t time, const char *details) {
    if (!drag_log_file) return;
    fprintf(drag_log_file, "[%lu] %s: %s->%s | drag=%s->%s | edge=%s->%s | motion=(%+.2f,%+.2f) | count=%lu | %s\n",
            (unsigned long)(time / 1000), event, state_to_string(fsm.previous_state),
            state_to_string(fsm.current_state), fsm.was_dragging ? "T" : "F", fsm.is_dragging ? "T" : "F",
            edge_to_string(fsm.previous_edge), edge_to_string(fsm.current_edge), fsm.motion_dx, fsm.motion_dy,
            (unsigned long)fsm.continuous_motion_count, details ? details : "");
    fflush(drag_log_file);
}

static void
calculate_motion_vector(uint32_t edge, double *dx, double *dy) {
    *dx = 0.0; *dy = 0.0;
    if (edge & EDGE_LEFT) *dx = -1.0;
    else if (edge & EDGE_RIGHT) *dx = 1.0;
    if (edge & EDGE_TOP) *dy = -1.0;
    else if (edge & EDGE_BOTTOM) *dy = 1.0;
    double mag = sqrt((*dx) * (*dx) + (*dy) * (*dy));
    if (mag > 0) { *dx /= mag; *dy /= mag; }
}

static void
inject_accumulated_motion(struct tp_dispatch *tp, uint64_t time) {
    if (fsm.last_motion_time == 0) { fsm.last_motion_time = time; return; }
    uint64_t time_since_last = time - fsm.last_motion_time;
    double dist_mm = EDGE_MOTION_CONFIG_SPEED_MM_S * ((double)time_since_last / 1000000.0);
    if (dist_mm < 0.001) return;

    struct device_float_coords raw = {
        .x = fsm.motion_dx * dist_mm * tp->accel.x_scale_coeff,
        .y = fsm.motion_dy * dist_mm * tp->accel.y_scale_coeff
    };
    struct normalized_coords delta = filter_dispatch(tp->device->pointer.filter, &raw, tp, time);
    pointer_notify_motion(&tp->device->base, time, &delta, &raw);

    fsm.last_motion_time = time;
    fsm.continuous_motion_count++;
}

static uint32_t
detect_touch_edge(const struct tp_dispatch *tp, const struct tp_touch *t) {
    uint32_t edge = EDGE_NONE;
    struct phys_coords mm = {EDGE_MOTION_CONFIG_EDGE_THRESHOLD_MM, EDGE_MOTION_CONFIG_EDGE_THRESHOLD_MM};
    struct device_coords threshold = evdev_device_mm_to_units(tp->device, &mm);

    if (t->point.x < threshold.x) edge |= EDGE_LEFT;
    if (t->point.x > tp->device->abs.absinfo_x->maximum - threshold.x) edge |= EDGE_RIGHT;
    if (t->point.y < threshold.y) edge |= EDGE_TOP;
    if (t->point.y > tp->device->abs.absinfo_y->maximum - threshold.y) edge |= EDGE_BOTTOM;
    return edge;
}

static enum edge_motion_state
calculate_next_state(bool is_dragging, uint32_t edge, enum edge_motion_state current) {
    if (!is_dragging) return STATE_IDLE;
    switch (current) {
        case STATE_IDLE:
        case STATE_DRAG_ACTIVE_CENTERED:
        case STATE_DRAG_EDGE_EXIT:
            return (edge != EDGE_NONE) ? STATE_DRAG_EDGE_ENTRY : STATE_DRAG_ACTIVE_CENTERED;
        case STATE_DRAG_EDGE_ENTRY:
        case STATE_DRAG_EDGE_CONTINUOUS:
            return (edge != EDGE_NONE) ? STATE_DRAG_EDGE_CONTINUOUS : STATE_DRAG_EDGE_EXIT;
    }
    return STATE_IDLE;
}

static void
tp_edge_motion_handle_timeout(uint64_t now, void *data) {
    struct edge_motion_fsm *fsm_ptr = data;

    // **FIXED LOGIC**: This now correctly handles the initial and continuous states.
    if (fsm_ptr->current_state != STATE_DRAG_EDGE_ENTRY &&
        fsm_ptr->current_state != STATE_DRAG_EDGE_CONTINUOUS) {
        return;
    }

    inject_accumulated_motion(fsm_ptr->tp, now);
    libinput_timer_set(&fsm_ptr->timer, now + EDGE_MOTION_CONFIG_MIN_INTERVAL_US);
}

void
tp_edge_motion_init(struct tp_dispatch *tp) {
    if (fsm.tp) return;
    memset(&fsm, 0, sizeof(fsm));
    fsm.current_state = STATE_IDLE;
    fsm.tp = tp;
    libinput_timer_init(&fsm.timer, tp_libinput_context(tp), "edge drag motion",
                        tp_edge_motion_handle_timeout, &fsm);
}

void
tp_edge_motion_cleanup(void) {
    if (drag_log_file) {
        // **FIXED**: Removed dependency on libinput_now() during cleanup.
        log_fsm_detailed("CLEANUP", 0, "Shutting down FSM");
        fclose(drag_log_file);
        drag_log_file = NULL;
    }
    if (fsm.tp) {
        libinput_timer_destroy(&fsm.timer);
    }
    memset(&fsm, 0, sizeof(fsm));
    fsm.current_state = STATE_IDLE;
}

int
tp_edge_motion_handle_drag_state(struct tp_dispatch *tp, uint64_t time) {
    if (!fsm.tp) tp_edge_motion_init(tp);

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

    uint32_t detected_edge = EDGE_NONE;
    struct tp_touch *t;
    if (drag_active) {
        tp_for_each_touch(tp, t) {
            if (t->state != TOUCH_NONE && t->state != TOUCH_HOVERING) {
                detected_edge = detect_touch_edge(tp, t);
                break;
            }
        }
    }

    if (!drag_log_file) {
        drag_log_file = fopen("/tmp/libinput-tap-drag-enhanced.log", "a");
        if(drag_log_file) { fprintf(drag_log_file, "\n=== NEW SESSION ===\n"); fflush(drag_log_file); }
    }

    fsm.previous_state = fsm.current_state;
    fsm.current_edge = detected_edge;
    fsm.is_dragging = drag_active;
    enum edge_motion_state next_state = calculate_next_state(drag_active, detected_edge, fsm.current_state);

    if (next_state != fsm.current_state) {
        fsm.current_state = next_state;
        fsm.state_entry_time = time;
        if (fsm.current_state != STATE_DRAG_EDGE_CONTINUOUS) fsm.continuous_motion_count = 0;
        log_fsm_detailed("STATE_TRANSITION", time, "");
    }

    switch (fsm.current_state) {
        case STATE_IDLE:
        case STATE_DRAG_ACTIVE_CENTERED:
        case STATE_DRAG_EDGE_EXIT:
            libinput_timer_cancel(&fsm.timer);
            break;
        case STATE_DRAG_EDGE_ENTRY:
            calculate_motion_vector(fsm.current_edge, &fsm.motion_dx, &fsm.motion_dy);
            fsm.last_motion_time = time;
            tp_edge_motion_handle_timeout(time, &fsm); // Kick-starts the timer loop
            break;
        case STATE_DRAG_EDGE_CONTINUOUS:
            calculate_motion_vector(fsm.current_edge, &fsm.motion_dx, &fsm.motion_dy);
            break;
    }
    return (fsm.current_state != STATE_DRAG_ACTIVE_CENTERED && fsm.current_state != STATE_IDLE);
}
