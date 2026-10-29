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
        /* Failed outer probes cannot steer, mark, conceal loss, or add noise. */
        LineSensor sensor; LineSensor_Reset(&sensor);
        for (unsigned t = 0; t < 30; ++t) {
            uint8_t ends = (t & 1u) ? 0x81u : 0u;
            LineFrame masked = Line_Describe((uint8_t)(0x66u | ends), (uint8_t)(0x66u | ends), 1, 0, t);
            assert(masked.black == 2 && masked.error == 0 && !masked.marker);
            LineSensor_Update(&sensor, (uint8_t)(0x66u | ends), t * 10u);
            if (t >= 2) assert(sensor.frame.valid && !sensor.frame.noisy && sensor.frame.error == 0);
        }
        LineFrame ends_only = Line_Describe(0x7eu, 0x7eu, 1, 0, 10);
        assert(ends_only.black == 0 && !Line_Trackable(&ends_only, 10, 0));
        LineFrame mark = Line_Describe(0xc1u, 0xc1u, 1, 0, 10);
        assert(mark.marker && Line_Trackable(&mark, 10, 1) && !Line_Trackable(&mark, 10, 0));
        LineFrame white = Line_Describe(0xffu, 0xe7u, 1, 0, 10);
        assert(!Line_Trackable(&white, 10, 0)); /* old stable line cannot conceal raw loss */
        assert(!Line_Trackable(&center, 51, 0));
        center.noisy = 1; assert(!Line_Trackable(&center, 10, 0));
    } else if (!strcmp(argv[1], "direction")) {
        LineCommand left = LineController_Step(&c, -3.5f, 10, 1, 15);
        assert(left.turn_rpm == 4 && left.base_rpm == 10);
        LineController_Reset(&c);
        LineCommand right = LineController_Step(&c, 3.5f, 10, 1, 15);
        assert(right.turn_rpm == -4);
        LineController_Reset(&c);
        assert(LineController_Step(&c, -3.5f, 10, -1, 15).turn_rpm == -4);
    } else if (!strcmp(argv[1], "speed_plan")) {
        float previous = 10;
        for (unsigned i = 0; i <= 300; ++i) {
            LineCommand out = LineController_Step(&c, 0, i * 10u, 1, 15);
            assert(out.base_rpm >= previous && out.base_rpm - previous <= .02001f && out.base_rpm <= 15);
            assert(out.base_rpm - out.turn_rpm > 0 && out.base_rpm + out.turn_rpm <= 20);
            previous = out.base_rpm;
        }
        assert(previous == 15);
        for (unsigned i = 301; i <= 330; ++i) {
            LineCommand out = LineController_Step(&c, 3.5f, i * 10u, 1, 15);
            assert(out.base_rpm <= previous && previous - out.base_rpm <= .20001f && out.base_rpm >= 10);
            assert(fabsf(out.turn_rpm) <= 4);
            previous = out.base_rpm;
        }
        assert(previous == 10);
        for (unsigned i = 331; i <= 700; ++i) LineController_Step(&c, 0, i * 10u, 1, 15);
        assert(c.base_rpm == 15);
    } else if (!strcmp(argv[1], "returning_pd")) {
        LineController_Step(&c, 2, 0, 1, 15);
        LineCommand returning = LineController_Step(&c, 1, 10, 1, 15);
        LineController_Reset(&c);
        LineController_Step(&c, 1, 0, 1, 15);
        LineCommand unchanged = LineController_Step(&c, 1, 10, 1, 15);
        assert(returning.turn_rpm < 0 && fabsf(returning.turn_rpm) < fabsf(unchanged.turn_rpm));
    } else if (!strcmp(argv[1], "invalid")) {
        assert(LineController_Step(&c, NAN, 0, 1, 15).base_rpm == 0);
        assert(LineController_Step(&c, 0, 0, 0, 15).base_rpm == 0);
        assert(LineController_Step(&c, 0, 0, 1, 21).base_rpm == 0);
    } else if (!strcmp(argv[1], "time_and_ceiling")) {
        for (unsigned i = 0; i < 400; ++i) {
            LineCommand out = LineController_Step(&c, 0, 0xfffffff0u + i * 10u, 1, 12);
            assert(out.base_rpm >= 10 && out.base_rpm <= 12);
        }
        assert(c.base_rpm == 12);
        LineCommand out = LineController_Step(&c, 0, c.tick_ms + 100u, 1, 12);
        assert(out.base_rpm == 10 && out.turn_rpm == 0); /* stale control history restarts conservatively */
    } else assert(0);
    printf("line_follow_%s: PASS\n", argv[1]); return 0;
}
