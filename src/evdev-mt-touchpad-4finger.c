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


/* Logging function to write swipe events to a file in /tmp */
void
tp_deal_with_it(struct tp_dispatch *tp, uint64_t time, const char *event_type, int finger_count, const struct normalized_coords *delta)
{
    FILE *log_file;
    char *log_path = "/tmp/touchpad_swipe.log";

    /* Determine swipe direction based on delta */
    const char *direction;
    if (fabs(delta->y) > fabs(delta->x) * 1.73) { /* 60-degree slope for vertical */
        direction = delta->y > 0 ? "down" : "up";
    } else {
        direction = delta->x > 0 ? "right" : "left";
    }

    /* Open log file in append mode */
    log_file = fopen(log_path, "a");
    if (log_file == NULL) {
        evdev_log_error(tp->device, "Failed to open log file %s\n", log_path);
        return;
    }

    /* Write human-readable log entry with deltas */
    fprintf(log_file, "%s: %d-finger swipe %s (delta x: %.2f, y: %.2f)\n",
            event_type, finger_count, direction, delta->x, delta->y);
    fclose(log_file);
}
