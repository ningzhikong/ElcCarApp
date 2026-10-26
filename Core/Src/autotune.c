#include "autotune.h"
#include "serial_router.h"
#include "wheel_calibration.h"
#include "drive_config.h"
#ifndef CAR_HOST_TEST
#include "usart.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TRACE_CAPACITY 112u
#define HEARTBEAT_MS 1000u
#define DURATION_MS 5500u
#define SAMPLE_MS 50u
#define LONG_DURATION_MS 16500u
#define LONG_SAMPLE_MS 150u
#define ARM_TIMEOUT_MS 5000u
#ifndef CAR_BUILD_ID
#define CAR_BUILD_ID "unidentified"
#endif
#define STATE_IDLE 0u
#define STATE_RUN 1u
#define STATE_DONE 2u
#define STATE_STOPPED 3u
#define STATE_FAULT 4u
#define FAULT_HEARTBEAT 10u
#define FAULT_UART 11u
#define FAULT_STALL 12u
#define FAULT_DIRECTION 13u
#define FAULT_SPEED 14u
#define FAULT_TRACE 15u
#define FAULT_FRAME 16u

typedef struct {
    uint32_t id;
    float kp_l, ki_l, kp_r, ki_r, peak, limit;
    uint8_t rpm_mode, hold_mode, long_hold;
} RunConfig;
typedef struct {
    uint32_t time_ms;
    int32_t raw_l, raw_r;
    float target_l, target_r, speed_l, speed_r, pwm_l, pwm_r;
    uint32_t period_us, flags;
} Sample;

static uint8_t enabled;
static volatile uint8_t armed, pending_run, ack_pending, io_error, start_expired;
static volatile uint32_t state, fault, count, elapsed, heartbeat_tick;
static uint32_t start_tick, next_sample, quiet_ticks, stall_l, stall_r, wrong_ticks;
static uint32_t no_motion_l, no_motion_r;
static volatile uint32_t arm_tick;
static RunConfig pending, active;
static int16_t pending_jog_l, pending_jog_r, jog_l, jog_r;
static uint32_t pending_jog_ms, jog_ms;
static uint8_t jog_active;
static Sample trace[TRACE_CAPACITY];
#ifndef CAR_GROUND_TEST
/* Stopped-only export: do not transmit at 100Hz during control. Keep the
 * legacy 50ms trace/protocol intact. Ground diagnostics have a separate log. */
#define RAW_CAPACITY 552u
#if AUTOTUNE_MAX_PWM > 999u
#error Diagnostic output must fit the hardware PWM range
#endif
typedef struct {
    uint16_t time_ms;
    int16_t raw_l, raw_r;
    uint16_t pwm_l, pwm_r, period_us;
} RawSample;
typedef char RawSampleSizeCheck[sizeof(RawSample) == 12u ? 1 : -1];
static RawSample raw_trace[RAW_CAPACITY];
static volatile uint32_t raw_count;
static uint32_t raw_next;
static int32_t raw_sum_l, raw_sum_r;
static uint32_t raw_period_sum;
static uint8_t raw_jog; /* 0: PI trace; 1: forward jog; 2: unsupported signed jog. */
#endif
static uint8_t rx_queue[256];
static volatile uint16_t rx_head, rx_tail;

static uint32_t lock(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    return saved;
}

void Autotune_SelectBootMode(void)
{
    GPIO_PinState first = HAL_GPIO_ReadPin(Start_GPIO_Port, Start_Pin);
    HAL_Delay(30u);
    enabled = first == GPIO_PIN_RESET &&
        HAL_GPIO_ReadPin(Start_GPIO_Port, Start_Pin) == GPIO_PIN_RESET;
}

int Autotune_IsEnabled(void) { return enabled != 0u; }

static void finish(uint32_t final_state, uint32_t reason)
{
    uint32_t saved = lock();
    Control_Stop();
    state = final_state;
    fault = reason;
    armed = pending_run = 0u;
    jog_active = 0u;
    __set_PRIMASK(saved);
}

void Autotune_OnUartError(void)
{
    io_error = 1u;
    Control_LatchFault(CONTROL_FAULT_UART);
}

void Autotune_OnByte(uint8_t byte)
{
    uint16_t next = (uint16_t)((rx_head + 1u) & 255u);
    if (next == rx_tail) Autotune_OnUartError();
    else {
        rx_queue[rx_head] = byte;
        rx_head = next;
    }
}

static int send_text(const char *text)
{
    if (HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 50u)
        != HAL_OK) {
        Autotune_OnUartError();
        finish(STATE_FAULT, FAULT_UART);
        return 0;
    }
    return 1;
}

void Autotune_Init(void)
{
    Control_Stop();
    Serial_StartAutotune();
    send_text("READY ELC_AUTOTUNE_V1\n");
}

static float target_at(uint32_t time_ms)
{
    if (time_ms < 500u || time_ms >= (active.long_hold ? 15500u : 4500u)) return 0.0f;
    if (active.hold_mode) return active.peak;
    if (time_ms < 1500u) return active.peak * 0.4f;
    if (time_ms < 3000u) return active.peak;
    return active.peak * 0.5f;
}

void Autotune_BeforeControl(uint32_t now_ms)
{
    if (armed && state != STATE_RUN && now_ms - arm_tick >= ARM_TIMEOUT_MS)
        armed = 0u;
    if (pending_run) {
        if (!armed) {
            finish(STATE_FAULT, 17u);
            start_expired = 1u;
            return;
        }
        jog_active = pending_run == 2u;
        active = pending;
        jog_l = pending_jog_l; jog_r = pending_jog_r; jog_ms = pending_jog_ms;
        pending_run = 0u;
        count = elapsed = next_sample = 0u;
#ifndef CAR_GROUND_TEST
        raw_count = raw_next = 0u;
        raw_sum_l = raw_sum_r = 0; raw_period_sum = 0u;
        raw_jog = jog_active ? ((jog_l >= 0 && jog_r >= 0) ? 1u : 2u) : 0u;
#endif
        stall_l = stall_r = wrong_ticks = 0u;
        no_motion_l = no_motion_r = 0u;
        fault = 0u;
        start_tick = heartbeat_tick = now_ms;
        if (!jog_active) {
            if (active.rpm_mode == 2u) Control_ConfigureDriveRPM();
            else {
                Control_ConfigurePI(active.kp_l, active.ki_l, active.kp_r, active.ki_r, active.limit);
                Control_SetRPMFeedback(active.rpm_mode);
            }
        }
        state = STATE_RUN;
        ack_pending = active.long_hold ? 7u : jog_active ? 2u : (active.rpm_mode == 2u ? (active.hold_mode ? 5u : 6u) :
                                       (active.hold_mode ? 4u : (active.rpm_mode ? 3u : 1u)));
    }
    if (state != STATE_RUN) return;
    elapsed = now_ms - start_tick;
    if (Control_GetFault()) finish(STATE_FAULT, Control_GetFault());
    else if (HAL_GPIO_ReadPin(ModeChange_GPIO_Port, ModeChange_Pin) == GPIO_PIN_RESET)
        finish(STATE_STOPPED, 0u);
    else if (now_ms - heartbeat_tick >= HEARTBEAT_MS)
        finish(STATE_FAULT, FAULT_HEARTBEAT);
    else if (elapsed >= (jog_active ? jog_ms : (active.long_hold ? LONG_DURATION_MS : DURATION_MS))) finish(STATE_DONE, 0u);
    else if (jog_active) Control_SetJogPWM(jog_l, jog_r);
    else {
        float target = target_at(elapsed);
        if (active.rpm_mode == 2u) Control_DriveRPM(target, 0.0f);
        else Control_SetTargets(active.rpm_mode ? Wheel_RPMToCounts(target, CAR_WHEEL_CPR_LEFT) : target,
                           active.rpm_mode ? Wheel_RPMToCounts(target, CAR_WHEEL_CPR_RIGHT) : target);
    }
}

void Autotune_AfterControl(const ControlView *v)
{
    Sample *s;
    if (v->raw_l == 0 && v->raw_r == 0) {
        if (quiet_ticks < 1000u) ++quiet_ticks;
    } else quiet_ticks = 0u;
    if (state == STATE_RUN) {
        if (v->raw_l > 100 || v->raw_l < -100 || v->raw_r > 100 || v->raw_r < -100) {
            finish(STATE_FAULT, FAULT_SPEED);
            return;
        }
        /* Jog establishes physical polarity: use a hard <=800ms duration
         * instead of feedback-dependent direction/stall checks. */
        if (!jog_active) {
            if ((v->target_l > 0.0f && v->raw_l < -2) ||
                (v->target_r > 0.0f && v->raw_r < -2)) ++wrong_ticks;
            else wrong_ticks = 0u;
            if (wrong_ticks >= 3u) {
                finish(STATE_FAULT, FAULT_DIRECTION);
                return;
            }
            /* A full-scale cap below 80 must not disable the fast stall guard. */
            float stall_pwm = active.limit < 80.0f ? active.limit * 0.8f : 80.0f;
            if (v->target_l > 0.0f && v->pwm_l >= stall_pwm && v->raw_l == 0) ++stall_l;
            else stall_l = 0u;
            if (v->target_r > 0.0f && v->pwm_r >= stall_pwm && v->raw_r == 0) ++stall_r;
            else stall_r = 0u;
            /* Includes zero gains and insufficient PWM: never finish a silent wheel. */
            if (v->target_l > 0.0f && v->raw_l <= 0) ++no_motion_l;
            else no_motion_l = 0u;
            if (v->target_r > 0.0f && v->raw_r <= 0) ++no_motion_r;
            else no_motion_r = 0u;
            if (stall_l >= 50u || stall_r >= 50u) finish(STATE_FAULT, FAULT_STALL);
            if (no_motion_l >= 100u || no_motion_r >= 100u) finish(STATE_FAULT, FAULT_STALL);
        }
    }
    if (state != STATE_RUN && state != STATE_DONE) return;
#ifndef CAR_GROUND_TEST
    /* The 15s profile sums every control tick into 30ms records. It does not
     * discard encoder counts or change the 10ms controller/guard cadence. */
    if (active.long_hold && raw_next <= LONG_DURATION_MS) {
        raw_sum_l += v->raw_l; raw_sum_r += v->raw_r;
        raw_period_sum += v->period_us;
    }
    if (raw_jog != 2u && elapsed >= raw_next && raw_next <= (active.long_hold ? LONG_DURATION_MS : DURATION_MS)) {
        if (raw_count >= RAW_CAPACITY) { finish(STATE_FAULT, FAULT_TRACE); return; }
        if ((active.long_hold ? raw_period_sum : v->period_us) > 65535u) {
            finish(STATE_FAULT, FAULT_TRACE); return;
        }
        RawSample *r = &raw_trace[raw_count];
        r->time_ms = (uint16_t)elapsed;
        r->raw_l = (int16_t)(active.long_hold ? raw_sum_l : v->raw_l);
        r->raw_r = (int16_t)(active.long_hold ? raw_sum_r : v->raw_r);
        r->pwm_l = (uint16_t)v->pwm_l; r->pwm_r = (uint16_t)v->pwm_r;
        r->period_us = (uint16_t)(active.long_hold ? raw_period_sum : v->period_us);
        raw_sum_l = raw_sum_r = 0; raw_period_sum = 0u;
        ++raw_count;
        raw_next = elapsed + (active.long_hold ? 30u : CONTROL_PERIOD_MS);
    }
#endif
    if (elapsed < next_sample || (state == STATE_DONE && next_sample > (active.long_hold ? LONG_DURATION_MS : DURATION_MS))) return;
    if (count >= TRACE_CAPACITY) {
        finish(STATE_FAULT, FAULT_TRACE);
        return;
    }
    s = &trace[count];
    s->time_ms = elapsed;
    s->raw_l = v->raw_l; s->raw_r = v->raw_r;
    s->target_l = v->target_l; s->target_r = v->target_r;
    s->speed_l = v->speed_l; s->speed_r = v->speed_r;
    s->pwm_l = v->pwm_l; s->pwm_r = v->pwm_r;
    s->period_us = v->period_us;
    s->flags = (v->pwm_l >= active.limit ? 1u : 0u) |
               (v->pwm_r >= active.limit ? 2u : 0u);
    ++count;
    next_sample += active.long_hold ? LONG_SAMPLE_MS : SAMPLE_MS;
}

static int number(const char *token, float low, float high, float *result)
{
    char *end;
    double value = strtod(token, &end);
    if (end == token || *end || !(value >= low && value <= high)) return 0;
    *result = (float)value;
    return 1;
}

static int request_id(const char *token, uint32_t *id)
{
    uint32_t n = 0u;
    if (!*token) return 0;
    while (*token) {
        if (*token < '0' || *token > '9' || n > 100000000u) return 0;
        n = n * 10u + (uint32_t)(*token++ - '0');
    }
    if (!n || n > 1000000000u) return 0;
    *id = n;
    return 1;
}

static void command(char *line)
{
    char *tokens[9], *token;
    char out[220];
    unsigned n = 0u;
    uint32_t saved;
    ControlView view;
    RunConfig config;
    for (token = strtok(line, " \t"); token && n < 9u; token = strtok(NULL, " \t"))
        tokens[n++] = token;
    if (!n) return;
    if (token || n == 9u) { send_text("ERR COMMAND\n"); return; }
    if (n == 1u && !strcmp(tokens[0], "HELLO")) {
        send_text("HELLO ELC_AUTOTUNE_V1 INC_PI_COUNTS_10MS 10 50 5500\n");
    } else if (n == 1u && !strcmp(tokens[0], "INFO")) {
        send_text("INFO ELC_INC_PI_V1 " CAR_BUILD_ID "\n");
    } else if (n == 1u && !strcmp(tokens[0], "STOP")) {
        saved = lock();
        ack_pending = 0u;
        start_expired = 0u;
        finish(STATE_STOPPED, 0u);
        __set_PRIMASK(saved);
        send_text("STOPPED\n");
    } else if (n == 1u && !strcmp(tokens[0], "PING")) {
        heartbeat_tick = HAL_GetTick();
    } else if (n == 1u && !strcmp(tokens[0], "STATUS")) {
        uint32_t id, saved_state, saved_fault, saved_count, saved_elapsed;
        unsigned saved_armed, stopped;
        saved = lock();
        id = active.id; saved_state = state; saved_fault = fault;
        saved_count = count; saved_elapsed = elapsed;
        saved_armed = armed; stopped = quiet_ticks >= 30u;
        __set_PRIMASK(saved);
        snprintf(out, sizeof out, "STATUS %lu %lu %lu %lu %lu %u %u\n",
            (unsigned long)id, (unsigned long)saved_state, (unsigned long)saved_fault,
            (unsigned long)saved_count, (unsigned long)saved_elapsed, saved_armed, stopped);
        send_text(out);
    } else if (n == 1u && !strcmp(tokens[0], "VIEW")) {
        Control_ReadView(&view);
        snprintf(out, sizeof out, "VIEW %lu %ld %ld %.4f %.4f %lu %lu\n",
            (unsigned long)view.tick_ms, (long)view.raw_l, (long)view.raw_r,
            (double)view.speed_l, (double)view.speed_r,
            (unsigned long)view.period_us, (unsigned long)view.fault);
        send_text(out);
    } else if (n == 1u && !strcmp(tokens[0], "RPMINFO")) {
        send_text("RPMINFO ELC_RPM_PI_ADAPTIVE_V1\n");
    } else if (n == 1u && !strcmp(tokens[0], "DRIVEINFO")) {
        snprintf(out, sizeof out, "DRIVEINFO ELC_DRIVE_RPM_V1 %.6f %.6f %.6f %.6f %.4f %u %d %d\n",
            (double)CAR_DRIVE_KP_LEFT, (double)CAR_DRIVE_KI_LEFT,
            (double)CAR_DRIVE_KP_RIGHT, (double)CAR_DRIVE_KI_RIGHT,
            (double)CAR_DRIVE_MAX_RPM, CAR_DRIVE_PWM_LIMIT, CAR_ENCODER_SIGN_LEFT, CAR_ENCODER_SIGN_RIGHT);
        send_text(out);
    } else if (n == 1u && !strcmp(tokens[0], "CAL")) {
        snprintf(out, sizeof out, "CAL MANUAL_REF_V1 %u %u\n",
            CAR_WHEEL_CPR_LEFT, CAR_WHEEL_CPR_RIGHT);
        send_text(out);
    } else if (n == 1u && !strcmp(tokens[0], "ENC")) {
        Control_ReadView(&view);
        snprintf(out, sizeof out, "ENC %lu %ld %ld\n", (unsigned long)view.tick_ms,
            (long)view.position_l, (long)view.position_r);
        send_text(out);
    } else if (n == 3u && !strcmp(tokens[0], "ARM")) {
        int left, right;
        if ((strcmp(tokens[1], "1") && strcmp(tokens[1], "-1")) ||
            (strcmp(tokens[2], "1") && strcmp(tokens[2], "-1"))) {
            send_text("ERR SIGNS\n"); return;
        }
        left = tokens[1][0] == '-' ? -1 : 1;
        right = tokens[2][0] == '-' ? -1 : 1;
        saved = lock();
        if (state == STATE_RUN || pending_run || quiet_ticks < 30u || io_error) {
            __set_PRIMASK(saved);
            send_text("ERR NOT_STOPPED\n"); return;
        }
        Control_ClearFault();
        Control_SetEncoderSigns(left, right);
        state = STATE_IDLE; fault = 0u; arm_tick = HAL_GetTick(); armed = 1u;
        __set_PRIMASK(saved);
        snprintf(out, sizeof out, "ARMED %d %d\n", left, right);
        send_text(out);
    } else if (n == 5u && !strcmp(tokens[0], "JOG")) {
        float left, right, duration;
        memset(&config, 0, sizeof config);
        if (!request_id(tokens[1], &config.id) ||
            !number(tokens[2], -150.0f, 150.0f, &left) ||
            !number(tokens[3], -150.0f, 150.0f, &right) ||
            !number(tokens[4], 50.0f, 800.0f, &duration) ||
            left != (float)(int)left || right != (float)(int)right ||
            duration != (float)(unsigned)duration || (unsigned)duration % 10u != 0u ||
            (left != 0.0f && right != 0.0f) || (left == 0.0f && right == 0.0f)) {
            send_text("ERR JOG\n"); return;
        }
        saved = lock();
        if (!armed || state == STATE_RUN || pending_run || quiet_ticks < 30u ||
            Control_GetFault() || io_error) {
            __set_PRIMASK(saved); send_text("ERR NOT_ARMED\n"); return;
        }
        config.limit = 150.0f;
        pending = config;
        pending_jog_l = (int16_t)left; pending_jog_r = (int16_t)right;
        pending_jog_ms = (uint32_t)duration;
        pending_run = 2u;
        __set_PRIMASK(saved);
    } else if (n == 8u && (!strcmp(tokens[0], "RUN") || !strcmp(tokens[0], "RUNRPM") ||
                           !strcmp(tokens[0], "HOLDRPM") || !strcmp(tokens[0], "HOLDRPM15") || !strcmp(tokens[0], "DRIVERPM") ||
                           !strcmp(tokens[0], "DRIVESTEP"))) {
        config.long_hold = !strcmp(tokens[0], "HOLDRPM15");
        config.hold_mode = config.long_hold || !strcmp(tokens[0], "HOLDRPM") || !strcmp(tokens[0], "DRIVERPM");
        config.rpm_mode = !strcmp(tokens[0], "DRIVERPM") || !strcmp(tokens[0], "DRIVESTEP") ? 2u :
                          (config.hold_mode || !strcmp(tokens[0], "RUNRPM"));
        if (!request_id(tokens[1], &config.id) ||
            !number(tokens[2], 0.0f, 10.0f, &config.kp_l) ||
            !number(tokens[3], 0.0f, 2.0f, &config.ki_l) ||
            !number(tokens[4], 0.0f, 10.0f, &config.kp_r) ||
            !number(tokens[5], 0.0f, 2.0f, &config.ki_r) ||
            !number(tokens[6], 1.0f, config.rpm_mode ? AUTOTUNE_MAX_RPM : 25.0f, &config.peak) ||
            !number(tokens[7], 20.0f, config.rpm_mode ? (float)AUTOTUNE_MAX_PWM : 200.0f, &config.limit) ||
            config.limit != (float)(unsigned)config.limit) {
            send_text("ERR GAINS\n"); return;
        }
        if (config.rpm_mode &&
            (!(Wheel_RPMToCounts(config.peak, CAR_WHEEL_CPR_LEFT) >= 1.0f &&
               Wheel_RPMToCounts(config.peak, CAR_WHEEL_CPR_LEFT) <= AUTOTUNE_MAX_TARGET_COUNTS) ||
             !(Wheel_RPMToCounts(config.peak, CAR_WHEEL_CPR_RIGHT) >= 1.0f &&
               Wheel_RPMToCounts(config.peak, CAR_WHEEL_CPR_RIGHT) <= AUTOTUNE_MAX_TARGET_COUNTS))) {
            send_text("ERR RPM\n"); return;
        }
        if (config.long_hold && (config.limit > 150.0f || config.peak > 30.0f)) {
            send_text("ERR LONG_PROFILE\n"); return;
        }
        if (config.rpm_mode == 2u && (config.kp_l != CAR_DRIVE_KP_LEFT || config.ki_l != CAR_DRIVE_KI_LEFT ||
            config.kp_r != CAR_DRIVE_KP_RIGHT || config.ki_r != CAR_DRIVE_KI_RIGHT ||
            config.limit != CAR_DRIVE_PWM_LIMIT || config.peak > CAR_DRIVE_MAX_RPM)) {
            send_text("ERR DRIVE_PROFILE\n"); return;
        }
        saved = lock();
        if (!armed || state == STATE_RUN || pending_run || quiet_ticks < 30u ||
            Control_GetFault() || io_error) {
            __set_PRIMASK(saved);
            send_text("ERR NOT_ARMED\n"); return;
        }
        pending = config;
        pending_run = 1u;
        __set_PRIMASK(saved);
        /* ACK is emitted by Poll only after the timer applies this entire set. */
#ifndef CAR_GROUND_TEST
    } else if (n == 1u && !strcmp(tokens[0], "LONGINFO")) {
        send_text("LONGINFO ELC_HOLD15_V1 150 16500 30\n");
    } else if (n == 1u && !strcmp(tokens[0], "RAWINFO")) {
        send_text("RAWINFO ELC_RAW10_V1 10 552\n");
    } else if (n == 1u && !strcmp(tokens[0], "JOGRAWINFO")) {
        send_text("JOGRAWINFO ELC_JOG_RAW10_V1 10 800\n");
    } else if (n == 1u && !strcmp(tokens[0], "RAWDUMP")) {
        uint32_t length, id;
        saved = lock();
        if (state == STATE_RUN || pending_run || armed || state == STATE_IDLE || jog_active) {
            __set_PRIMASK(saved); send_text("ERR BUSY\n"); return;
        }
        length = raw_count; id = active.id;
        __set_PRIMASK(saved);
        if (raw_jog == 2u) { send_text("ERR RAW_MODE\n"); return; }
        if (raw_jog) snprintf(out, sizeof out, "RAWJOG %lu %lu %d %d %lu\n", (unsigned long)id,
            (unsigned long)length, (int)jog_l, (int)jog_r, (unsigned long)jog_ms);
        else snprintf(out, sizeof out, "%s %lu %lu %u %u %.4f\n", active.long_hold ? "RAWBEGIN15" : "RAWBEGIN", (unsigned long)id,
            (unsigned long)length, active.rpm_mode, active.hold_mode, (double)active.peak);
        if (!send_text(out)) return;
        for (uint32_t index = 0; index < length; ++index) {
            const RawSample *r = &raw_trace[index];
            snprintf(out, sizeof out, "%u,%d,%d,%u,%u,%lu\n", (unsigned)r->time_ms,
                (int)r->raw_l, (int)r->raw_r, (unsigned)r->pwm_l, (unsigned)r->pwm_r,
                (unsigned long)r->period_us);
            if (!send_text(out)) return;
        }
        send_text("RAWEND\n");
#endif
    } else if (n == 1u && !strcmp(tokens[0], "DUMP")) {
        uint32_t index, length, id;
        saved = lock();
        if (state == STATE_RUN || pending_run || state == STATE_IDLE) {
            __set_PRIMASK(saved); send_text("ERR BUSY\n"); return;
        }
        length = count; id = active.id;
        __set_PRIMASK(saved);
        snprintf(out, sizeof out, "BEGIN %lu %lu\n", (unsigned long)id, (unsigned long)length);
        if (!send_text(out)) return;
        for (index = 0u; index < length; ++index) {
            const Sample *s = &trace[index];
            snprintf(out, sizeof out, "%lu,%ld,%ld,%.4f,%.4f,%.4f,%.4f,%.0f,%.0f,%lu,%lu\n",
                (unsigned long)s->time_ms, (long)s->raw_l, (long)s->raw_r,
                (double)s->target_l, (double)s->target_r,
                (double)s->speed_l, (double)s->speed_r,
                (double)s->pwm_l, (double)s->pwm_r,
                (unsigned long)s->period_us, (unsigned long)s->flags);
            if (!send_text(out)) return;
        }
        send_text("END\n");
    } else send_text("ERR COMMAND\n");
}

void Autotune_Poll(void)
{
    static char line[128];
    static uint16_t length;
    static uint8_t discard;
    uint8_t byte;
    char out[150];
    uint32_t saved;
    if (io_error) {
        finish(STATE_FAULT, FAULT_UART);
        saved = lock();
        rx_tail = rx_head; io_error = 0u;
        __set_PRIMASK(saved);
        length = 0u; discard = 1u;
        Serial_StartAutotune();
    }
    if (start_expired) {
        start_expired = 0u;
        send_text("ERR ARM_EXPIRED\n");
    }
    if (ack_pending) {
        RunConfig applied;
        uint8_t ack_kind;
        int16_t applied_l, applied_r;
        uint32_t applied_ms;
        saved = lock();
        applied = active; ack_kind = ack_pending; ack_pending = 0u;
        applied_l = jog_l; applied_r = jog_r; applied_ms = jog_ms;
        __set_PRIMASK(saved);
        if (ack_kind == 2u) {
            snprintf(out, sizeof out, "JOGACK %lu %d %d %lu\n",
                (unsigned long)applied.id, (int)applied_l, (int)applied_r,
                (unsigned long)applied_ms);
        } else {
        snprintf(out, sizeof out, "%s %lu %.6f %.6f %.6f %.6f %.4f %.0f\n",
            ack_kind == 7u ? "HOLD15ACK" : ack_kind == 6u ? "DRIVESTEPACK" : (ack_kind == 5u ? "DRIVEACK" :
            (ack_kind == 4u ? "HOLDACK" : (ack_kind == 3u ? "RPMACK" : "ACK"))), (unsigned long)applied.id,
            (double)applied.kp_l, (double)applied.ki_l,
            (double)applied.kp_r, (double)applied.ki_r,
            (double)applied.peak, (double)applied.limit);
        }
        send_text(out);
    }
    /* Bounded work so a noisy peer cannot monopolize the foreground. */
    for (unsigned budget = 0u; budget < 64u && rx_tail != rx_head; ++budget) {
        byte = rx_queue[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1u) & 255u);
        if (byte == '\n') {
            if (!discard) {
                if (length && line[length - 1u] == '\r') --length;
                line[length] = '\0'; command(line);
            } else send_text("ERR FRAME\n");
            length = 0u; discard = 0u;
        } else if (byte == 0u || byte > 127u || length >= sizeof line - 1u) {
            finish(STATE_FAULT, FAULT_FRAME);
            discard = 1u;
        } else if (!discard) line[length++] = (char)byte;
    }
}
