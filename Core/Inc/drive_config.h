#ifndef CAR_DRIVE_CONFIG_H
#define CAR_DRIVE_CONFIG_H
/* Raised-wheel baseline, not a ground-load autotuning result. PI error is
 * still wheel-specific counts/10ms; only the public drive command is RPM. */
#define CAR_DRIVE_KP_LEFT 2.0f
#define CAR_DRIVE_KI_LEFT 1.0f
#define CAR_DRIVE_KP_RIGHT 2.0f
#define CAR_DRIVE_KI_RIGHT 1.0f
#define CAR_DRIVE_MAX_RPM 20.0f
#define CAR_DRIVE_PWM_LIMIT 150u
#endif
