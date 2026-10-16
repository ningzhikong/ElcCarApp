#ifndef CAR_ROUTE_CONFIG_H
#define CAR_ROUTE_CONFIG_H
/* Initial PCB/track values; calibrate these against the physical course. */
#define ROUTE_GRAY_BLACK 2800u
#define ROUTE_GRAY_LINE 2600u
#define ROUTE_MARK_MIN_COUNTS 20u
#define ROUTE_MARK_COOLDOWN_MS 500u
#define ROUTE_LAP_MIN_COUNTS 3000u
#define ROUTE_PHASE_TIMEOUT_MS 10000u
#define ROUTE_LINE_LOST_MS 400u
#define ROUTE_IMU_MAX_GAP_MS 50u
#define ROUTE_ABCD_SPEED 10.0f
#define ROUTE_ACBD_SPEED 25.0f
#define ROUTE_LINE_KP 22.0f
#define ROUTE_LINE_KD 8.0f
#define ROUTE_HEADING_KP 22.0f
#define ROUTE_HEADING_KD 8.0f
#define ROUTE_MAX_TURN 18.0f
#endif
