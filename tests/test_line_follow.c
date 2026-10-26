#include "line_follow.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
#ifdef _WIN32
    _set_error_mode(_OUT_TO_STDERR);
#endif
    assert(argc == 2);
    LineController c;
    LineController_Reset(&c);
    if (!strcmp(argv[1], "classification")) {
        LineFrame center = Line_Describe(0xe7u, 0xe7u, 1, 0, 10);
        assert(center.black == 2 && center.error == 0 && !center.marker && Line_Trackable(&center, 10, 0));
        LineFrame mark = Line_Describe(0xe0u, 0xe0u, 1, 0, 10);
        assert(mark.marker && Line_Trackable(&mark, 10, 1) && !Line_Trackable(&mark, 10, 0));
        LineFrame white = Line_Describe(0xffu, 0xe7u, 1, 0, 10);
        assert(!Line_Trackable(&white, 10, 0)); /* old stable line cannot conceal raw loss */
        assert(!Line_Trackable(&center, 51, 0));
        center.noisy = 1; assert(!Line_Trackable(&center, 10, 0));
    } else if (!strcmp(argv[1], "direction")) {
        LineCommand left = LineController_Step(&c, -3.5f, 10, 1, ROUTE_LINE_FAST_RPM);
        assert(left.turn_rpm == 4 && left.base_rpm == ROUTE_LINE_SLOW_RPM);
        LineController_Reset(&c);
        LineCommand right = LineController_Step(&c, 3.5f, 10, 1, ROUTE_LINE_FAST_RPM);
        assert(right.turn_rpm == -4);
        LineController_Reset(&c);
        assert(LineController_Step(&c, -3.5f, 10, -1, ROUTE_LINE_FAST_RPM).turn_rpm == -4);
    } else if (!strcmp(argv[1], "speed_plan")) {
        /* One control tick is 10ms, so these bound the per-tick slew. */
        const float accel_step = ROUTE_LINE_ACCEL_RPM_S * .01f + .00001f;
        const float decel_step = ROUTE_LINE_DECEL_RPM_S * .01f + .00001f;
        /* Enough ticks for the full slow->fast ramp, plus margin. */
        const unsigned ramp_ticks =
            (unsigned)((ROUTE_LINE_FAST_RPM - ROUTE_LINE_SLOW_RPM) / ROUTE_LINE_ACCEL_RPM_S * 100.0f) + 100u;
        float previous = ROUTE_LINE_SLOW_RPM;
        for (unsigned i = 0; i <= ramp_ticks; ++i) {
            LineCommand out = LineController_Step(&c, 0, i * 10u, 1, ROUTE_LINE_FAST_RPM);
            assert(out.base_rpm >= previous && out.base_rpm - previous <= accel_step &&
                   out.base_rpm <= ROUTE_LINE_FAST_RPM);
            assert(out.base_rpm - out.turn_rpm > 0 &&
                   out.base_rpm + out.turn_rpm <= ROUTE_LINE_FAST_RPM + ROUTE_MAX_TURN_RPM);
            previous = out.base_rpm;
        }
        assert(previous == ROUTE_LINE_FAST_RPM);
        /* Enough ticks for the full fast->slow deceleration, plus margin. */
        const unsigned brake_ticks =
            (unsigned)((ROUTE_LINE_FAST_RPM - ROUTE_LINE_SLOW_RPM) / ROUTE_LINE_DECEL_RPM_S * 100.0f) + 20u;
        for (unsigned i = ramp_ticks + 1; i <= ramp_ticks + brake_ticks; ++i) {
            LineCommand out = LineController_Step(&c, 3.5f, i * 10u, 1, ROUTE_LINE_FAST_RPM);
            assert(out.base_rpm <= previous && previous - out.base_rpm <= decel_step &&
                   out.base_rpm >= ROUTE_LINE_SLOW_RPM);
            assert(fabsf(out.turn_rpm) <= ROUTE_MAX_TURN_RPM);
            previous = out.base_rpm;
        }
        assert(previous == ROUTE_LINE_SLOW_RPM);
        /* Re-accelerate from the floor back to the ceiling. */
        for (unsigned i = ramp_ticks + brake_ticks + 1;
             i <= ramp_ticks + brake_ticks + ramp_ticks; ++i)
            LineController_Step(&c, 0, i * 10u, 1, ROUTE_LINE_FAST_RPM);
        assert(c.base_rpm == ROUTE_LINE_FAST_RPM);
    } else if (!strcmp(argv[1], "returning_pd")) {
        LineController_Step(&c, 2, 0, 1, ROUTE_LINE_FAST_RPM);
        LineCommand returning = LineController_Step(&c, 1, 10, 1, ROUTE_LINE_FAST_RPM);
        LineController_Reset(&c);
        LineController_Step(&c, 1, 0, 1, ROUTE_LINE_FAST_RPM);
        LineCommand unchanged = LineController_Step(&c, 1, 10, 1, ROUTE_LINE_FAST_RPM);
        assert(returning.turn_rpm < 0 && fabsf(returning.turn_rpm) < fabsf(unchanged.turn_rpm));
    } else if (!strcmp(argv[1], "invalid")) {
        assert(LineController_Step(&c, NAN, 0, 1, ROUTE_LINE_FAST_RPM).base_rpm == 0);
        assert(LineController_Step(&c, 0, 0, 0, ROUTE_LINE_FAST_RPM).base_rpm == 0);
        /* Above the ceiling: one RPM past the configured fast bound. */
        assert(LineController_Step(&c, 0, 0, 1, ROUTE_LINE_FAST_RPM + 1.0f).base_rpm == 0);
        /* Below the floor: one RPM under the configured slow bound. */
        assert(LineController_Step(&c, 0, 0, 1, ROUTE_LINE_SLOW_RPM - 1.0f).base_rpm == 0);
    } else if (!strcmp(argv[1], "time_and_ceiling")) {
        const float mid = .5f * (ROUTE_LINE_SLOW_RPM + ROUTE_LINE_FAST_RPM);
        for (unsigned i = 0; i < 400; ++i) {
            LineCommand out = LineController_Step(&c, 0, 0xfffffff0u + i * 10u, 1, mid);
            assert(out.base_rpm >= ROUTE_LINE_SLOW_RPM && out.base_rpm <= mid);
        }
        assert(c.base_rpm == mid);
        LineCommand out = LineController_Step(&c, 0, c.tick_ms + 100u, 1, mid);
        assert(out.base_rpm == ROUTE_LINE_SLOW_RPM && out.turn_rpm == 0); /* stale control history restarts conservatively */
    } else assert(0);
    printf("line_follow_%s: PASS\n", argv[1]); return 0;
}
