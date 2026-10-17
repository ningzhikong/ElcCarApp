#include "control_math.h"
#include <string.h>

int32_t Encoder_Delta16(uint16_t current, uint16_t previous)
{
    uint32_t delta = (uint16_t)(current - previous);
    return delta <= 32767u ? (int32_t)delta : (int32_t)delta - 65536;
}

float Encoder_Filter(float previous, int32_t delta)
{
    return previous + 0.125f * ((float)delta - previous);
}

void Encoder_WindowReset(EncoderWindow *window)
{
    memset(window, 0, sizeof *window);
}

float Encoder_WindowStep(EncoderWindow *window, int32_t delta, unsigned ticks)
{
    int32_t sum = 0;
    unsigned index;
    if (ticks < 1u) ticks = 1u;
    if (ticks > 25u) ticks = 25u;
    window->samples[window->next] = delta;
    if (++window->next == 25u) window->next = 0u;
    index = window->next;
    for (unsigned i = 0; i < ticks; ++i) {
        index = index ? index - 1u : 24u;
        sum += window->samples[index];
    }
    return (float)sum / (float)ticks;
}

void PID_Reset(Speed_PID_t *pid)
{
    pid->target = pid->actual = pid->err = 0.0f;
    pid->err_last = pid->err_prev = pid->output = 0.0f;
}

float PID_Incremental_Calc(Speed_PID_t *pid, float target, float actual)
{
    float delta;
    pid->target = target;
    pid->actual = actual;
    pid->err = target - actual;
    delta = pid->kp * (pid->err - pid->err_last) + pid->ki * pid->err
          + pid->kd * (pid->err - 2.0f * pid->err_last + pid->err_prev);
    if (delta > 30.0f) delta = 30.0f;
    if (delta < -30.0f) delta = -30.0f;
    pid->output += delta;
    pid->err_prev = pid->err_last;
    pid->err_last = pid->err;
    if (pid->output > pid->max_output) pid->output = pid->max_output;
    if (pid->output < -pid->max_output) pid->output = -pid->max_output;
    return pid->output;
}

float PID_Positional_Calc(Turn_PID_t *pid, float target, float actual)
{
    pid->err = target - actual;
    pid->output = pid->kp * pid->err + pid->kd * (pid->err - pid->err_last);
    pid->err_last = pid->err;
    if (pid->output > pid->max_output) pid->output = pid->max_output;
    if (pid->output < -pid->max_output) pid->output = -pid->max_output;
    return pid->output;
}
