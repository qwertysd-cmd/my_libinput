#ifndef EVDEV_MT_TOUCHPAD_TDS
#define EVDEV_MT_TOUCHPAD_TDS

#include "evdev-mt-touchpad.h"

extern int last_dragging_state;

int tp_edge_motion_handle_drag_state(struct tp_dispatch *tp, uint64_t time);

#endif /* EVDEV_MT_TOUCHPAD_TDS */
