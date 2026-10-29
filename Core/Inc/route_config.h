#ifndef CAR_ROUTE_CONFIG_H
#define CAR_ROUTE_CONFIG_H
/* Initial PCB/track values; calibrate these against the physical course. */
/* Physical check with d066e4f3e041a4aa showed inverted displayed colors:
 * the board outputs white=high, black=low.
 * Normalize once at acquisition: application/display white=1, black=0.
 * PULL jumper + PA2 pull-up input, not analog gray/ADC thresholds. */
#define IR_WHITE_RAW_LEVEL 1u
#if IR_WHITE_RAW_LEVEL != 0u && IR_WHITE_RAW_LEVEL != 1u
#error IR_WHITE_RAW_LEVEL must be zero or one
#endif
#define IR_NORMALIZE_LEVEL(raw) ((raw) == IR_WHITE_RAW_LEVEL ? 1u : 0u)
/* Verified physically: channel zero is the leftmost probe in forward view. */
#define IR_CHANNEL_DIRECTION 1
/* Physical channels 2..7 only; failed outer probes are never sampled. */
#define IR_ACTIVE_CHANNEL_MASK 0x7eu
#define IR_CHANNEL_ENABLED(i) ((IR_ACTIVE_CHANNEL_MASK & (1u << (i))) != 0u)
#define ROUTE_IR_MAX_AGE_MS 40u
#define ROUTE_MARK_MIN_BLACK 5u
#define ROUTE_MARK_MIN_COUNTS 20u
#define ROUTE_MARK_COOLDOWN_MS 500u
#define ROUTE_LAP_MIN_COUNTS 3000u
#define ROUTE_PHASE_TIMEOUT_MS 10000u
#define ROUTE_LINE_LOST_MS 400u
#define ROUTE_IMU_MAX_GAP_MS 50u
/* RPM at the wheel output. Conservative commissioning values; steering
 * gains now produce RPM, and still require track/load validation. */
#define ROUTE_LINE_SLOW_RPM 10.0f
#define ROUTE_LINE_FAST_RPM 15.0f
#define ROUTE_LINE_ACCEL_RPM_S 2.0f
#define ROUTE_LINE_DECEL_RPM_S 20.0f
#define ROUTE_LINE_D_TAU_S 0.05f
#define ROUTE_ABCD_RPM ROUTE_LINE_FAST_RPM
#define ROUTE_ACBD_RPM ROUTE_LINE_FAST_RPM
#define ROUTE_LINE_KP 2.0f
/* Derivative uses error/second, not the previous per-tick gain convention. */
#define ROUTE_LINE_KD 0.04f
#define ROUTE_HEADING_KP 4.0f
#define ROUTE_HEADING_KD 1.5f
#define ROUTE_MAX_TURN_RPM 4.0f
#endif
