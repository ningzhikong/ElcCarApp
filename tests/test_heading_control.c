#include "heading_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char **argv)
{
    HeadingController controller;
    HeadingCommand command;
    assert(argc == 2); Heading_Reset(&controller);
    if (!strcmp(argv[1], "direction")) {
        for (uint32_t t = 0; t <= 500; t += 10) command = Heading_Step(&controller, 5, 0, t);
        assert(command.valid && fabsf(command.turn_rpm + 3) < .0001f);
        for (uint32_t t = 510; t <= 1010; t += 10) command = Heading_Step(&controller, -5, 0, t);
        assert(command.valid && fabsf(command.turn_rpm - 3) < .0001f);
    } else if (!strcmp(argv[1], "clamp_slew")) {
        float last = 0;
        for (uint32_t t = 0; t <= 500; t += 10) {
            command = Heading_Step(&controller, 20, 0, t);
            assert(command.valid && command.turn_rpm >= -6 && fabsf(command.turn_rpm - last) <= .20001f);
            last = command.turn_rpm;
        }
        assert(command.turn_rpm == -6);
    } else if (!strcmp(argv[1], "damping")) {
        for (uint32_t t = 0; t <= 100; t += 10) command = Heading_Step(&controller, 0, 10, t);
        assert(command.valid && fabsf(command.turn_rpm + 1.2f) < .0001f);
    } else if (!strcmp(argv[1], "invalid")) {
        const float invalid[][2] = {{NAN, 0}, {0, NAN}, {26, 0}, {-26, 0}, {0, 181}, {0, -181}};
        for (unsigned i = 0; i < sizeof invalid / sizeof invalid[0]; ++i) {
            command = Heading_Step(&controller, invalid[i][0], invalid[i][1], 0);
            assert(!command.valid && command.turn_rpm == 0 && !controller.seen);
        }
    } else if (!strcmp(argv[1], "gap")) {
        assert(Heading_Step(&controller, 5, 0, 0).valid);
        assert(!Heading_Step(&controller, 5, 0, 51).valid);
    } else if (!strcmp(argv[1], "wrap")) {
        assert(Heading_Step(&controller, 5, 0, 0xfffffff8u).valid);
        command = Heading_Step(&controller, 5, 0, 2u);
        assert(command.valid && fabsf(command.turn_rpm + .4f) < .0001f);
    } else if (!strcmp(argv[1], "reset")) {
        Heading_Step(&controller, 5, 0, 100); Heading_Reset(&controller);
        assert(!controller.seen && controller.turn == 0 && controller.tick == 0);
    } else assert(0);
    printf("heading_%s: PASS\n", argv[1]); return 0;
}
