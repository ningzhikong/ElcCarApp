#ifndef CAR_DRIVE_CONFIG_H
#define CAR_DRIVE_CONFIG_H
/* Raised-wheel baseline, not a ground-load autotuning result. PI error is
 * still wheel-specific counts/10ms; only the public drive command is RPM. */
#define CAR_DRIVE_KP_LEFT 2.0f
#define CAR_DRIVE_KI_LEFT 1.0f
#define CAR_DRIVE_KP_RIGHT 2.0f
#define CAR_DRIVE_KI_RIGHT 1.0f
#if defined(CAR_GROUND_TEST) && !defined(CAR_GROUND_LINE)
/* Dedicated straight-line load test; normal routes retain their baseline. */
#define CAR_DRIVE_MAX_RPM 130.0f
#define CAR_DRIVE_PWM_LIMIT 450u
#else
#define CAR_DRIVE_MAX_RPM 40.0f
/* Matches the older firmware that drove this chassis: the incremental PI
 * needs this much headroom to overcome static friction and reach the target
 * counts. A 150 limit left the wheels stalled. */
#define CAR_DRIVE_PWM_LIMIT 400u
#endif
#endif
