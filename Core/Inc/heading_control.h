#ifndef CAR_HEADING_CONTROL_H
#define CAR_HEADING_CONTROL_H
#include <stdint.h>

typedef struct { uint32_t tick; float turn; uint8_t seen; } HeadingController;
typedef struct { float turn_rpm; uint8_t valid; } HeadingCommand;
extern const volatile float heading_kp, heading_kd, heading_max_turn_rpm;
extern const volatile float heading_slew_rpm_s, heading_max_error_deg;
void Heading_Reset(HeadingController *controller);
/* Positive error/rate mean left yaw; positive wheel turn commands left. */
HeadingCommand Heading_Step(HeadingController *controller, float error_deg, float rate_dps, uint32_t now);
#endif
