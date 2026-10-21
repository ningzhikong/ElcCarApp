#ifndef CAR_LINE_FOLLOW_H
#define CAR_LINE_FOLLOW_H
#include "ir_filter.h"
#include "route_config.h"

typedef struct {
    uint32_t raw, levels, tick_ms;
    unsigned black, raw_black, valid, noisy, marker;
    float error;
} LineFrame;
typedef struct { IRFilter filter; LineFrame frame; } LineSensor;
typedef struct {
    float base_rpm, last_error, derivative;
    uint32_t tick_ms;
    unsigned started;
} LineController;
typedef struct { float base_rpm, turn_rpm; } LineCommand;

void LineSensor_Reset(LineSensor *s);
void LineSensor_Update(LineSensor *s, uint8_t raw_levels, uint32_t now);
LineFrame Line_Describe(uint8_t raw_levels, uint8_t levels, unsigned valid, unsigned noisy, uint32_t now);
int Line_Trackable(const LineFrame *f, uint32_t now, int allow_marker);
void LineController_Reset(LineController *c);
LineCommand LineController_Step(LineController *c, float error, uint32_t now, int direction, float ceiling_rpm);
#endif
