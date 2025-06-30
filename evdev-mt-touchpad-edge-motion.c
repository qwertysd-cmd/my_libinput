/*
 * Edge Motion Implementation
 * Similar to edge scrolling but generates pointer motion when finger reaches edges during tap-and-drag
 */

#include "config.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#include "evdev-mt-touchpad.h"

/* Speed of edge motion in normalized coordinates per second */
#define DEFAULT_EDGE_MOTION_SPEED 0.5

/* How close to edge before triggering edge motion */
#define DEFAULT_EDGE_MOTION_THRESHOLD TP_MM_TO_DPI_NORMALIZED(3)

/* Delay before starting edge motion to avoid accidental triggering */
#define DEFAULT_EDGE_MOTION_DELAY ms2us(150)

/* Interval for generating motion events */
#define EDGE_MOTION_INTERVAL ms2us(16) /* ~60fps */

enum edge_motion_event {
	EDGE_MOTION_EVENT_TOUCH,
	EDGE_MOTION_EVENT_MOTION,
	EDGE_MOTION_EVENT_RELEASE,
	EDGE_MOTION_EVENT_TIMEOUT_START,
	EDGE_MOTION_EVENT_TIMEOUT_TICK,
	EDGE_MOTION_EVENT_POSTED,
};

enum tp_edge_motion_touch_state {
	EDGE_MOTION_TOUCH_STATE_NONE,
	EDGE_MOTION_TOUCH_STATE_DRAGGING,
	EDGE_MOTION_TOUCH_STATE_EDGE_NEW,
	EDGE_MOTION_TOUCH_STATE_EDGE_ACTIVE,
};

static inline const char*
edge_motion_state_to_str(enum tp_edge_motion_touch_state state)
{
	switch (state) {
	CASE_RETURN_STRING(EDGE_MOTION_TOUCH_STATE_NONE);
	CASE_RETURN_STRING(EDGE_MOTION_TOUCH_STATE_DRAGGING);
	CASE_RETURN_STRING(EDGE_MOTION_TOUCH_STATE_EDGE_NEW);
	CASE_RETURN_STRING(EDGE_MOTION_TOUCH_STATE_EDGE_ACTIVE);
	}
	return NULL;
}

static inline const char*
edge_motion_event_to_str(enum edge_motion_event event)
{
	switch (event) {
	CASE_RETURN_STRING(EDGE_MOTION_EVENT_TOUCH);
	CASE_RETURN_STRING(EDGE_MOTION_EVENT_MOTION);
	CASE_RETURN_STRING(EDGE_MOTION_EVENT_RELEASE);
	CASE_RETURN_STRING(EDGE_MOTION_EVENT_TIMEOUT_START);
	CASE_RETURN_STRING(EDGE_MOTION_EVENT_TIMEOUT_TICK);
	CASE_RETURN_STRING(EDGE_MOTION_EVENT_POSTED);
	}
	return NULL;
}

static uint32_t
tp_touch_get_edge_motion_edges(const struct tp_dispatch *tp, const struct tp_touch *t)
{
	uint32_t edges = EDGE_NONE;

	/* Check if we're close enough to edges to trigger edge motion */
	if (t->point.x > (tp->scroll.right_edge + DEFAULT_EDGE_MOTION_THRESHOLD))
		edges |= EDGE_RIGHT;

	if (t->point.y > (tp->scroll.bottom_edge + DEFAULT_EDGE_MOTION_THRESHOLD))
		edges |= EDGE_BOTTOM;

	if (t->point.x < DEFAULT_EDGE_MOTION_THRESHOLD)
		edges |= EDGE_LEFT;

	if (t->point.y < DEFAULT_EDGE_MOTION_THRESHOLD)
		edges |= EDGE_TOP;

	return edges;
}

static bool
tp_touch_is_tap_dragging(const struct tp_dispatch *tp, const struct tp_touch *t)
{
	/* TODO: Connect this to the actual tap-and-drag detection logic
	 * This should return true if the touch is currently in a tap-and-drag state
	 * You'll need to integrate with the existing tap detection code
	 */

	/* Placeholder logic - replace with actual tap-and-drag state check */
	return false; /* FIXME: implement actual tap-and-drag detection */
}

static void
tp_edge_motion_set_timer_start(struct tp_dispatch *tp, struct tp_touch *t, uint64_t time)
{
	/* Set timer for initial delay before starting edge motion */
	libinput_timer_set(&t->edge_motion.start_timer, time + DEFAULT_EDGE_MOTION_DELAY);
}

static void
tp_edge_motion_set_timer_tick(struct tp_dispatch *tp, struct tp_touch *t, uint64_t time)
{
	/* Set timer for next motion event */
	libinput_timer_set(&t->edge_motion.tick_timer, time + EDGE_MOTION_INTERVAL);
}

static void
tp_edge_motion_set_state(struct tp_dispatch *tp,
			 struct tp_touch *t,
			 enum tp_edge_motion_touch_state state,
			 uint64_t time)
{
	libinput_timer_cancel(&t->edge_motion.start_timer);
	libinput_timer_cancel(&t->edge_motion.tick_timer);

	t->edge_motion.state = state;

	switch (state) {
	case EDGE_MOTION_TOUCH_STATE_NONE:
		t->edge_motion.edges = EDGE_NONE;
		break;
	case EDGE_MOTION_TOUCH_STATE_DRAGGING:
		t->edge_motion.edges = EDGE_NONE;
		break;
	case EDGE_MOTION_TOUCH_STATE_EDGE_NEW:
		t->edge_motion.edges = tp_touch_get_edge_motion_edges(tp, t);
		tp_edge_motion_set_timer_start(tp, t, time);
		break;
	case EDGE_MOTION_TOUCH_STATE_EDGE_ACTIVE:
		tp_edge_motion_set_timer_tick(tp, t, time);
		break;
	}
}

static void
tp_edge_motion_handle_none(struct tp_dispatch *tp,
			   struct tp_touch *t,
			   enum edge_motion_event event,
			   uint64_t time)
{
	switch (event) {
	case EDGE_MOTION_EVENT_TOUCH:
		if (tp_touch_is_tap_dragging(tp, t)) {
			tp_edge_motion_set_state(tp, t, EDGE_MOTION_TOUCH_STATE_DRAGGING, time);
		}
		break;
	case EDGE_MOTION_EVENT_MOTION:
	case EDGE_MOTION_EVENT_RELEASE:
	case EDGE_MOTION_EVENT_TIMEOUT_START:
	case EDGE_MOTION_EVENT_TIMEOUT_TICK:
	case EDGE_MOTION_EVENT_POSTED:
		/* Ignore these events in NONE state */
		break;
	}
}

static void
tp_edge_motion_handle_dragging(struct tp_dispatch *tp,
			       struct tp_touch *t,
			       enum edge_motion_event event,
			       uint64_t time)
{
	switch (event) {
	case EDGE_MOTION_EVENT_TOUCH:
		/* Already in dragging state */
		break;
	case EDGE_MOTION_EVENT_MOTION:
		if (tp_touch_get_edge_motion_edges(tp, t)) {
			tp_edge_motion_set_state(tp, t, EDGE_MOTION_TOUCH_STATE_EDGE_NEW, time);
		}
		break;
	case EDGE_MOTION_EVENT_RELEASE:
		tp_edge_motion_set_state(tp, t, EDGE_MOTION_TOUCH_STATE_NONE, time);
		break;
	case EDGE_MOTION_EVENT_TIMEOUT_START:
	case EDGE_MOTION_EVENT_TIMEOUT_TICK:
	case EDGE_MOTION_EVENT_POSTED:
		/* Ignore these events in DRAGGING state */
		break;
	}
}

static void
tp_edge_motion_handle_edge_new(struct tp_dispatch *tp,
			       struct tp_touch *t,
			       enum edge_motion_event event,
			       uint64_t time)
{
	switch (event) {
	case EDGE_MOTION_EVENT_TOUCH:
		/* Already in edge state */
		break;
	case EDGE_MOTION_EVENT_MOTION:
		/* Update edges, but stay in edge_new until timeout */
		t->edge_motion.edges = tp_touch_get_edge_motion_edges(tp, t);
		if (!t->edge_motion.edges) {
			tp_edge_motion_set_state(tp, t, EDGE_MOTION_TOUCH_STATE_DRAGGING, time);
		}
		break;
	case EDGE_MOTION_EVENT_RELEASE:
		tp_edge_motion_set_state(tp, t, EDGE_MOTION_TOUCH_STATE_NONE, time);
		break;
	case EDGE_MOTION_EVENT_TIMEOUT_START:
		tp_edge_motion_set_state(tp, t, EDGE_MOTION_TOUCH_STATE_EDGE_ACTIVE, time);
		break;
	case EDGE_MOTION_EVENT_TIMEOUT_TICK:
	case EDGE_MOTION_EVENT_POSTED:
		/* Ignore these events in EDGE_NEW state */
		break;
	}
}

static void
tp_edge_motion_handle_edge_active(struct tp_dispatch *tp,
				  struct tp_touch *t,
				  enum edge_motion_event event,
				  uint64_t time)
{
	switch (event) {
	case EDGE_MOTION_EVENT_TOUCH:
		/* Already in edge state */
		break;
	case EDGE_MOTION_EVENT_MOTION:
		/* Update edges */
		t->edge_motion.edges = tp_touch_get_edge_motion_edges(tp, t);
		if (!t->edge_motion.edges) {
			tp_edge_motion_set_state(tp, t, EDGE_MOTION_TOUCH_STATE_DRAGGING, time);
		}
		break;
	case EDGE_MOTION_EVENT_RELEASE:
		tp_edge_motion_set_state(tp, t, EDGE_MOTION_TOUCH_STATE_NONE, time);
		break;
	case EDGE_MOTION_EVENT_TIMEOUT_START:
		/* Ignore - already active */
		break;
	case EDGE_MOTION_EVENT_TIMEOUT_TICK:
		/* Generate motion and reset timer */
		tp_edge_motion_set_timer_tick(tp, t, time);
		break;
	case EDGE_MOTION_EVENT_POSTED:
		break;
	}
}

static void
tp_edge_motion_handle_event(struct tp_dispatch *tp,
			    struct tp_touch *t,
			    enum edge_motion_event event,
			    uint64_t time)
{
	enum tp_edge_motion_touch_state current = t->edge_motion.state;

	switch (current) {
	case EDGE_MOTION_TOUCH_STATE_NONE:
		tp_edge_motion_handle_none(tp, t, event, time);
		break;
	case EDGE_MOTION_TOUCH_STATE_DRAGGING:
		tp_edge_motion_handle_dragging(tp, t, event, time);
		break;
	case EDGE_MOTION_TOUCH_STATE_EDGE_NEW:
		tp_edge_motion_handle_edge_new(tp, t, event, time);
		break;
	case EDGE_MOTION_TOUCH_STATE_EDGE_ACTIVE:
		tp_edge_motion_handle_edge_active(tp, t, event, time);
		break;
	}

	if (current != t->edge_motion.state)
		evdev_log_debug(tp->device,
				"edge-motion: touch %d state %s → %s → %s\n",
				t->index,
				edge_motion_state_to_str(current),
				edge_motion_event_to_str(event),
				edge_motion_state_to_str(t->edge_motion.state));
}

static void
tp_edge_motion_handle_timeout_start(uint64_t now, void *data)
{
	struct tp_touch *t = data;
	tp_edge_motion_handle_event(t->tp, t, EDGE_MOTION_EVENT_TIMEOUT_START, now);
}

static void
tp_edge_motion_handle_timeout_tick(uint64_t now, void *data)
{
	struct tp_touch *t = data;
	tp_edge_motion_handle_event(t->tp, t, EDGE_MOTION_EVENT_TIMEOUT_TICK, now);
}

void
tp_edge_motion_init(struct tp_dispatch *tp, struct evdev_device *device)
{
	struct tp_touch *t;
	int i = 0;

	tp_for_each_touch(tp, t) {
		char timer_name[64];

		snprintf(timer_name, sizeof(timer_name),
			 "%s (%d) edgemotion-start",
			 evdev_device_get_sysname(device), i);
		libinput_timer_init(&t->edge_motion.start_timer,
				    tp_libinput_context(tp),
				    timer_name,
				    tp_edge_motion_handle_timeout_start, t);

		snprintf(timer_name, sizeof(timer_name),
			 "%s (%d) edgemotion-tick",
			 evdev_device_get_sysname(device), i);
		libinput_timer_init(&t->edge_motion.tick_timer,
				    tp_libinput_context(tp),
				    timer_name,
				    tp_edge_motion_handle_timeout_tick, t);

		t->edge_motion.state = EDGE_MOTION_TOUCH_STATE_NONE;
		t->edge_motion.edges = EDGE_NONE;
		i++;
	}
}

void
tp_remove_edge_motion(struct tp_dispatch *tp)
{
	struct tp_touch *t;

	tp_for_each_touch(tp, t) {
		libinput_timer_cancel(&t->edge_motion.start_timer);
		libinput_timer_destroy(&t->edge_motion.start_timer);
		libinput_timer_cancel(&t->edge_motion.tick_timer);
		libinput_timer_destroy(&t->edge_motion.tick_timer);
	}
}

void
tp_edge_motion_handle_state(struct tp_dispatch *tp, uint64_t time)
{
	struct tp_touch *t;

	tp_for_each_touch(tp, t) {
		if (!t->dirty)
			continue;

		switch (t->state) {
		case TOUCH_NONE:
		case TOUCH_HOVERING:
			break;
		case TOUCH_BEGIN:
			tp_edge_motion_handle_event(tp, t, EDGE_MOTION_EVENT_TOUCH, time);
			break;
		case TOUCH_UPDATE:
			tp_edge_motion_handle_event(tp, t, EDGE_MOTION_EVENT_MOTION, time);
			break;
		case TOUCH_MAYBE_END:
			/* This shouldn't happen we transfer to TOUCH_END before processing state */
			evdev_log_debug(tp->device,
					"touch %d: unexpected state %d\n",
					t->index, t->state);
			_fallthrough_;
		case TOUCH_END:
			tp_edge_motion_handle_event(tp, t, EDGE_MOTION_EVENT_RELEASE, time);
			break;
		}
	}
}

int
tp_edge_motion_post_events(struct tp_dispatch *tp, uint64_t time)
{
	struct evdev_device *device = tp->device;
	struct tp_touch *t;
	struct normalized_coords motion = { 0.0, 0.0 };
	double speed_per_frame = DEFAULT_EDGE_MOTION_SPEED * (EDGE_MOTION_INTERVAL / 1000000.0);

	tp_for_each_touch(tp, t) {
		if (t->edge_motion.state != EDGE_MOTION_TOUCH_STATE_EDGE_ACTIVE)
			continue;

		if (!t->edge_motion.edges)
			continue;

		/* Calculate motion based on active edges */
		motion.x = 0.0;
		motion.y = 0.0;

		if (t->edge_motion.edges & EDGE_RIGHT)
			motion.x += speed_per_frame;
		if (t->edge_motion.edges & EDGE_LEFT)
			motion.x -= speed_per_frame;
		if (t->edge_motion.edges & EDGE_BOTTOM)
			motion.y += speed_per_frame;
		if (t->edge_motion.edges & EDGE_TOP)
			motion.y -= speed_per_frame;

		/* Normalize diagonal motion to maintain consistent speed */
		if ((t->edge_motion.edges & (EDGE_LEFT | EDGE_RIGHT)) &&
		    (t->edge_motion.edges & (EDGE_TOP | EDGE_BOTTOM))) {
			double factor = 1.0 / sqrt(2.0);
			motion.x *= factor;
			motion.y *= factor;
		}

		/* TODO: Send the motion event using the appropriate libinput API
		 * This should generate a pointer motion event
		 * Replace with actual API call:
		 */
		/* evdev_notify_motion(device, time, &motion); */

		evdev_log_debug(device,
				"edge-motion: generating motion x=%.2f y=%.2f for edges 0x%x\n",
				motion.x, motion.y, t->edge_motion.edges);

		tp_edge_motion_handle_event(tp, t, EDGE_MOTION_EVENT_POSTED, time);
	}

	return 0;
}

void
tp_edge_motion_stop_events(struct tp_dispatch *tp, uint64_t time)
{
	struct tp_touch *t;

	tp_for_each_touch(tp, t) {
		if (t->edge_motion.state != EDGE_MOTION_TOUCH_STATE_NONE) {
			tp_edge_motion_set_state(tp, t, EDGE_MOTION_TOUCH_STATE_NONE, time);
		}
	}
}

/* You'll also need to add these fields to the tp_touch structure:
 *
 * struct {
 *     enum tp_edge_motion_touch_state state;
 *     uint32_t edges;
 *     struct libinput_timer start_timer;
 *     struct libinput_timer tick_timer;
 * } edge_motion;
 */
