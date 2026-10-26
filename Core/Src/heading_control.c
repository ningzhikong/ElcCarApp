#include "heading_control.h"
#include <string.h>

const volatile float heading_kp = .6f;
const volatile float heading_kd = .12f;
const volatile float heading_max_turn_rpm = 6.0f;
const volatile float heading_slew_rpm_s = 20.0f;
const volatile float heading_max_error_deg = 25.0f;

void Heading_Reset(HeadingController *controller) { memset(controller, 0, sizeof *controller); }

HeadingCommand Heading_Step(HeadingController *controller, float error, float rate, uint32_t now)
{
    HeadingCommand command = {0.0f, 0u};
    uint32_t dt = controller->seen ? now - controller->tick : 10u;
    float desired, step;
    if (!(error >= -heading_max_error_deg && error <= heading_max_error_deg &&
          rate >= -180.0f && rate <= 180.0f) || dt > 50u) {
        Heading_Reset(controller); return command;
    }
    desired = -heading_kp * error - heading_kd * rate;
    if (desired > heading_max_turn_rpm) desired = heading_max_turn_rpm;
    if (desired < -heading_max_turn_rpm) desired = -heading_max_turn_rpm;
    step = heading_slew_rpm_s * (float)dt * .001f;
    if (desired > controller->turn + step) desired = controller->turn + step;
    if (desired < controller->turn - step) desired = controller->turn - step;
    controller->turn = desired; controller->tick = now; controller->seen = 1u;
    command.turn_rpm = desired; command.valid = 1u;
    return command;
}
