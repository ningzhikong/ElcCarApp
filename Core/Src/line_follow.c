#include "line_follow.h"
#include <math.h>

static float bound(float value, float low, float high)
{ return value < low ? low : value > high ? high : value; }

LineFrame Line_Describe(uint8_t raw, uint8_t levels, unsigned valid, unsigned noisy, uint32_t now)
{
    LineFrame f = {0};
    float weighted = 0.0f;
    raw |= (uint8_t)~IR_ACTIVE_CHANNEL_MASK;
    levels |= (uint8_t)~IR_ACTIVE_CHANNEL_MASK;
    f.raw = raw; f.levels = levels; f.tick_ms = now;
    f.valid = valid; f.noisy = noisy;
    for (unsigned i = 0; i < 8; ++i) {
        if (!IR_CHANNEL_ENABLED(i)) continue;
        if (!(raw & (1u << i))) ++f.raw_black;
        if (!(levels & (1u << i))) { ++f.black; weighted += (float)i - 3.5f; }
    }
    f.error = f.black ? weighted / (float)f.black : 0.0f;
    /* A centered normal line spanning two sensors is not a route marker. */
    f.marker = valid && !noisy && f.black >= ROUTE_MARK_MIN_BLACK && f.raw_black >= ROUTE_MARK_MIN_BLACK;
    return f;
}

void LineSensor_Reset(LineSensor *s) { memset(s, 0, sizeof *s); }
void LineSensor_Update(LineSensor *s, uint8_t raw, uint32_t now)
{
    raw |= (uint8_t)~IR_ACTIVE_CHANNEL_MASK;
    IRFilter_Step(&s->filter, raw, now);
    s->frame = Line_Describe(raw, s->filter.levels, s->filter.initialized == 0xffu, s->filter.noisy, now);
}

int Line_Trackable(const LineFrame *f, uint32_t now, int allow_marker)
{
    if (!f->valid || f->noisy || now - f->tick_ms > ROUTE_IR_MAX_AGE_MS || !f->raw_black) return 0;
    if (allow_marker && f->marker) return 1;
    return f->raw_black <= 4u && f->black >= 1u && f->black <= 4u;
}

void LineController_Reset(LineController *c)
{ memset(c, 0, sizeof *c); c->base_rpm = ROUTE_LINE_SLOW_RPM; }

LineCommand LineController_Step(LineController *c, float error, uint32_t now, int direction, float ceiling)
{
    LineCommand out = {0.0f, 0.0f};
    if (!(error >= -3.5f && error <= 3.5f) || (direction != 1 && direction != -1) ||
        !(ceiling >= ROUTE_LINE_SLOW_RPM && ceiling <= ROUTE_LINE_FAST_RPM)) {
        LineController_Reset(c); return out;
    }
    uint32_t elapsed = now - c->tick_ms;
    if (!c->started || elapsed < 5u || elapsed > ROUTE_IR_MAX_AGE_MS) {
        c->base_rpm = ROUTE_LINE_SLOW_RPM;
        c->derivative = 0.0f;
    } else {
        float dt = (float)elapsed * .001f;
        float derivative = (error - c->last_error) / dt;
        c->derivative += dt / (ROUTE_LINE_D_TAU_S + dt) * (derivative - c->derivative);
        float severity = fmaxf(fabsf(error) / 1.5f, fabsf(c->derivative) / 12.0f);
        float wanted = ROUTE_LINE_SLOW_RPM + (ceiling - ROUTE_LINE_SLOW_RPM) * (1.0f - bound(severity, 0.0f, 1.0f));
        /* Speed reductions are faster than acceleration. Fault stops bypass this. */
        c->base_rpm += bound(wanted - c->base_rpm, -ROUTE_LINE_DECEL_RPM_S * dt, ROUTE_LINE_ACCEL_RPM_S * dt);
    }
    c->started = 1u; c->tick_ms = now; c->last_error = error;
    out.base_rpm = bound(c->base_rpm, ROUTE_LINE_SLOW_RPM, ceiling);
    float turn = -(float)direction * (ROUTE_LINE_KP * error + ROUTE_LINE_KD * c->derivative);
    out.turn_rpm = bound(turn, -ROUTE_MAX_TURN_RPM, ROUTE_MAX_TURN_RPM);
    return out;
}
