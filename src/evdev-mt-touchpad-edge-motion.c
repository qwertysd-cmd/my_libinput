#include "evdev-mt-touchpad-tds.h"
#include "inttypes.h"
#include <stdio.h>

static FILE *drag_log_file = NULL;
int last_dragging_state = 0; /* 0 for not dragging, 1 for dragging */

int
tp_edge_motion_handle_drag_state(struct tp_dispatch *tp, uint64_t time)
{
    int is_dragging = 0; /* 0 for not dragging, 1 for dragging */

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

    if (is_dragging != last_dragging_state) {
        if (!drag_log_file) {
            drag_log_file = fopen("/tmp/libinput-tap-drag.log", "a");
            if (!drag_log_file) {
                evdev_log_error(tp->device, "Failed to open log file /tmp/libinput-tap-drag.log\n");
                return is_dragging;
            }
        }

        fprintf(drag_log_file, "[%" PRIu64 "] %s\n",
                time,
                is_dragging ? "started drag" : "stopped drag");
        fflush(drag_log_file);

        last_dragging_state = is_dragging;
    }

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
