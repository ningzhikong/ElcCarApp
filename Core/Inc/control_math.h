#ifndef CAR_CONTROL_MATH_H
#define CAR_CONTROL_MATH_H
#include <stdint.h>

typedef struct {
    float kp, ki, kd;
    float target, actual, err, err_last, err_prev, output, max_output;
} Speed_PID_t;
typedef struct {
    float kp, kd, err, err_last, output, max_output;
} Turn_PID_t;

int32_t Encoder_Delta16(uint16_t current, uint16_t previous);
float Encoder_Filter(float previous, int32_t delta);
typedef struct {
    int32_t samples[25];
    unsigned next;
} EncoderWindow;
void Encoder_WindowReset(EncoderWindow *window);
/* Delta is a signed 16-bit timer difference, optionally sign-inverted. */
float Encoder_WindowStep(EncoderWindow *window, int32_t delta, unsigned ticks);
void PID_Reset(Speed_PID_t *pid);
float PID_Incremental_Calc(Speed_PID_t *pid, float target, float actual);
float PID_Positional_Calc(Turn_PID_t *pid, float target, float actual);
#endif
