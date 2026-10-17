#ifndef CAR_WHEEL_CALIBRATION_H
#define CAR_WHEEL_CALIBRATION_H
/* Wheel-output counts per revolution, using the current x4 encoder mode.
 * Manual reference values (not manufacturer specifications): left 5400/5
 * and 1072/1; right 6586/5 and 1320/1. Recheck after changing hardware. */
#define CAR_WHEEL_CPR_LEFT 1080u
#define CAR_WHEEL_CPR_RIGHT 1320u
#if CAR_WHEEL_CPR_LEFT == 0 || CAR_WHEEL_CPR_RIGHT == 0
#error Wheel counts per revolution must be positive
#endif
static inline float Wheel_RPMToCounts(float rpm, unsigned cpr)
{
    return rpm * (float)cpr / 6000.0f;
}
#endif
