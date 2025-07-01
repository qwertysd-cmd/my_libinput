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
static int last_dragging_state = 0; /* 0 for not dragging, 1 for dragging */
static uint32_t last_edge = EDGE_NONE; /* Last edge(s) detected */

/* Inject pointer motion based on edge state */
static void
tp_inject_pointer_motion(struct tp_dispatch *tp, uint64_t time, double dx, double dy)
{
    struct device_float_coords raw;
    struct normalized_coords delta;
    const double speed_mm_s = 20.0; /* Fixed speed, adjustable */
    uint64_t time_delta_us = ms2us(12); /* Assume 12ms frame interval */

    /* Normalize direction vector */
    double magnitude = sqrt(dx * dx + dy * dy);
    if (magnitude > 0) {
        dx /= magnitude;
        dy /= magnitude;
    } else {
        dx = 0.0;
        dy = 0.0; /* No motion if no direction */
    }

    /* Convert speed to device units */
    raw.x = dx * speed_mm_s * time_delta_us / 1000000.0 * tp->accel.x_scale_coeff;
    raw.y = dy * speed_mm_s * time_delta_us / 1000000.0 * tp->accel.y_scale_coeff;

    /* Apply pointer acceleration filter */
    delta = filter_dispatch(tp->device->pointer.filter, &raw, tp, time);

    /* Post pointer motion event */
    pointer_notify_motion(&tp->device->base, time, &delta, &raw);

    /* Log for debugging */
    evdev_log_debug(tp->device, "Injected pointer motion: dx=%f, dy=%f, speed=%f mm/s\n",
                    dx, dy, speed_mm_s);
}

/* Extend edge detection to include all four edges if not initialized */
static uint32_t
tp_touch_get_extended_edge(const struct tp_dispatch *tp, const struct tp_touch *t)
{
    uint32_t edge = 0; /* Initialize to EDGE_NONE */
    struct phys_coords mm = { 7.0, 7.0 }; /* 7mm threshold for edges */
    struct device_coords edge_threshold;

    /* Convert 7mm to device coordinates */
    edge_threshold = evdev_device_mm_to_units(tp->device, &mm);

    /* Check left edge */
    if (tp->scroll.left_edge == 0) {
        if (t->point.x < edge_threshold.x)
            edge |= EDGE_LEFT;
    } else {
        if (t->point.x < tp->scroll.left_edge)
            edge |= EDGE_LEFT;
    }

    /* Check top edge */
    if (tp->scroll.upper_edge == 0) {
        if (t->point.y < edge_threshold.y)
            edge |= EDGE_TOP;
    } else {
        if (t->point.y < tp->scroll.upper_edge)
            edge |= EDGE_TOP;
    }

    /* Check right edge */
    if (tp->scroll.right_edge == 0) {
        int32_t max_x = tp->device->abs.absinfo_x->maximum;
        if (t->point.x > max_x - edge_threshold.x)
            edge |= EDGE_RIGHT;
    } else {
        if (t->point.x > tp->scroll.right_edge)
            edge |= EDGE_RIGHT;
    }

    /* Check bottom edge */
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

/* Convert edge flags to string for logging */
static const char *
edge_to_string(uint32_t edge)
{
    switch (edge) {
    case EDGE_LEFT:
        return "left";
    case EDGE_RIGHT:
        return "right";
    case EDGE_TOP:
        return "top";
    case EDGE_BOTTOM:
        return "bottom";
    case EDGE_LEFT | EDGE_TOP:
        return "top left";
    case EDGE_LEFT | EDGE_BOTTOM:
        return "bottom left";
    case EDGE_RIGHT | EDGE_TOP:
        return "top right";
    case EDGE_RIGHT | EDGE_BOTTOM:
        return "bottom right";
    default:
        return "none";
    }
}

int
tp_edge_motion_handle_drag_state(struct tp_dispatch *tp, uint64_t time)
{
    int is_dragging = 0; /* 0 for not dragging, 1 for dragging */
    uint32_t current_edge = EDGE_NONE;
    struct tp_touch *t;
    bool edge_changed = false;
    double dx = 0.0, dy = 0.0;

    /* Check tap-and-drag state */
    switch (tp->tap.state) {
    case TAP_STATE_1FGTAP_DRAGGING:
    case TAP_STATE_1FGTAP_DRAGGING_2:
    case TAP_STATE_1FGTAP_DRAGGING_WAIT:
    case TAP_STATE_1FGTAP_DRAGGING_OR_TAP:
    case TAP_STATE_1FGTAP_DRAGGING_OR_DOUBLETAP:
        is_dragging = 1;
        break;
    default:
        is_dragging = 0;
        break;
    }

    /* Find the first active touch for edge detection */
    tp_for_each_touch(tp, t) {
        if (t->state == TOUCH_NONE || t->state == TOUCH_HOVERING || t->palm.state != PALM_NONE)
            continue;
        current_edge = tp_touch_get_extended_edge(tp, t);
        break; /* Use the first valid touch */
    }

    /* Open log file if not already open */
    if (!drag_log_file) {
        drag_log_file = fopen("/tmp/libinput-tap-drag.log", "a");
        if (!drag_log_file) {
            evdev_log_error(tp->device, "Failed to open log file /tmp/libinput-tap-drag.log\n");
            return is_dragging;
        }
    }

    /* Handle drag state changes */
    if (is_dragging != last_dragging_state) {
        fprintf(drag_log_file, "%s\n",
                is_dragging ? "started drag" : "stopped drag");
        fflush(drag_log_file);
        last_dragging_state = is_dragging;
        edge_changed = true; /* Trigger edge/centered logging on drag state change */
    }

    /* Handle edge-specific and centered logging, and set motion direction */
    if (is_dragging) {
        if (current_edge != last_edge || edge_changed) {
            if (current_edge != EDGE_NONE) {
                /* Log "moving [edge(s)]" if on an edge */
                fprintf(drag_log_file, "moving %s\n", edge_to_string(current_edge));
            } else {
                /* Log "centered" if not on an edge */
                fprintf(drag_log_file, "centered\n");
            }
            fflush(drag_log_file);
        }

        /* Set dx, dy based on edge */
        if (current_edge & EDGE_LEFT)
            dx = -1.0;
        else if (current_edge & EDGE_RIGHT)
            dx = 1.0;
        if (current_edge & EDGE_TOP)
            dy = -1.0;
        else if (current_edge & EDGE_BOTTOM)
            dy = 1.0;

        /* Inject pointer motion if dragging and direction is set */
        if (dx != 0.0 || dy != 0.0) {
            tp_inject_pointer_motion(tp, time, dx, dy);
        }
    } else if (last_edge != EDGE_NONE || edge_changed) {
        /* Log "centered" when drag stops and last position was on an edge */
        fprintf(drag_log_file, "centered\n");
        fflush(drag_log_file);
    }

    last_edge = current_edge;

    return is_dragging;
}

void
tp_edge_motion_cleanup(void)
{
    if (drag_log_file) {
        fclose(drag_log_file);
        drag_log_file = NULL;
    }
}
