#include "autotune.h"
#include "control_math.h"
#include "drive_config.h"
#include "wheel_calibration.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int huart1;
static uint32_t now, mask, control_fault;
static ControlView measured;
static int stop_key, configured;
static unsigned pwm_cap = 100u;
static float pwm_fraction = 1.0f;
static char output[32000];
static unsigned output_length;

uint32_t __get_PRIMASK(void) { return mask; }
void __disable_irq(void) { mask = 1u; }
void __set_PRIMASK(uint32_t m) { mask = m; }
uint32_t HAL_GetTick(void) { return now; }
void HAL_Delay(uint32_t ms) { now += ms; }
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin)
{
    (void)port;
    return pin == Start_Pin || stop_key ? GPIO_PIN_RESET : GPIO_PIN_SET;
}
int HAL_UART_Transmit(void *uart, uint8_t *data, uint16_t length, uint32_t timeout)
{
    (void)uart; (void)timeout;
    assert(!mask); /* No formatted transmission while the timer is masked. */
    assert(output_length + length < sizeof output);
    memcpy(output + output_length, data, length);
    output_length += length;
    output[output_length] = 0;
    return HAL_OK;
}
void Serial_StartAutotune(void) { }
void Control_Stop(void)
{
    measured.target_l = measured.target_r = measured.pwm_l = measured.pwm_r = 0.0f;
}
void Control_LatchFault(uint32_t reason) { control_fault = reason; Control_Stop(); }
uint32_t Control_GetFault(void) { return control_fault; }
void Control_ClearFault(void) { control_fault = 0u; Control_Stop(); }
void Control_ReadView(ControlView *view) { *view = measured; }
void Control_SetEncoderSigns(int l, int r) { assert(abs(l) == 1 && abs(r) == 1); }
void Control_SetRPMFeedback(int enabled) { assert(enabled == 0 || enabled == 1); }
void Control_ConfigurePI(float pl, float il, float pr, float ir, float limit)
{
    assert(pl == 2.0f && il == 0.3f && pr == 2.0f && ir == 0.3f && limit == (float)pwm_cap);
    ++configured;
    Control_Stop();
}
void Control_SetTargets(float l, float r)
{
    measured.target_l = l; measured.target_r = r;
    measured.pwm_l = l > 0.0f ? pwm_cap * pwm_fraction : 0.0f;
    measured.pwm_r = r > 0.0f ? pwm_cap * pwm_fraction : 0.0f;
}
void Control_ConfigureDriveRPM(void) { ++configured; pwm_cap = 150u; Control_Stop(); }
void Control_DriveRPM(float base, float turn)
{
    assert(turn == 0 && base >= 0 && base <= 20);
    Control_SetTargets(Wheel_RPMToCounts(base, CAR_WHEEL_CPR_LEFT), Wheel_RPMToCounts(base, CAR_WHEEL_CPR_RIGHT));
}
void Control_SetJogPWM(int16_t l, int16_t r)
{
    assert(abs(l) <= 150 && abs(r) <= 150 && !(l && r));
    measured.target_l = measured.target_r = 0.0f;
    measured.pwm_l = (float)l; measured.pwm_r = (float)r;
}

static void clear_output(void) { output_length = 0u; output[0] = 0; }
static void feed(const char *text)
{
    while (*text) Autotune_OnByte((uint8_t)*text++);
    Autotune_Poll();
    Autotune_Poll();
    Autotune_Poll();
}
static void exchange(const char *text, const char *expected)
{
    clear_output(); feed(text);
    assert(strstr(output, expected));
}
static void tick_pair(int left, int right)
{
    now += 10u;
    measured.tick_ms = now;
    measured.period_us = 10000u;
    measured.raw_l = left; measured.raw_r = right;
    measured.speed_l = (float)left; measured.speed_r = (float)right;
    Autotune_BeforeControl(now);
    Autotune_AfterControl(&measured);
}
static void tick(int raw) { tick_pair(raw, raw); }
static void arm(void)
{
    for (unsigned i = 0u; i < 35u; ++i) tick(0);
    exchange("ARM 1 -1\n", "ARMED 1 -1\n");
}
static void start(void)
{
    char command[128], ack[160];
    arm(); clear_output();
    snprintf(command, sizeof command, "RUN 7 2 .3 2 .3 5 %u\n", pwm_cap);
    feed(command);
    assert(!strstr(output, "ACK"));
    assert(configured == 0);
    tick(0); Autotune_Poll();
    assert(configured == 1);
    snprintf(ack, sizeof ack, "ACK 7 2.000000 0.300000 2.000000 0.300000 5.0000 %u\n", pwm_cap);
    assert(strstr(output, ack));
}

static void test_math(void)
{
    float filtered;
    Speed_PID_t pid;
    memset(&pid, 0, sizeof pid);
    assert(Encoder_Delta16(65531u, 0u) == -5);
    assert(Encoder_Delta16(2u, 65534u) == 4);
    assert(Encoder_Delta16(65534u, 2u) == -4);
    {
        EncoderWindow window;
        Encoder_WindowReset(&window);
        for (unsigned i = 0; i < 100; ++i) {
            float speed = Encoder_WindowStep(&window, i % 10 < 7 ? 5 : 0, 10);
            if (i >= 9) assert(speed == 3.5f);
        }
        Encoder_WindowReset(&window);
        for (unsigned i = 0; i < 10; ++i) Encoder_WindowStep(&window, -32768, 10);
        assert(Encoder_WindowStep(&window, -32768, 10) == -32768.0f);
        Encoder_WindowReset(&window);
        assert(Encoder_WindowStep(&window, 10, 10) == 1.0f);
        for (unsigned i = 0; i < 10; ++i) Encoder_WindowStep(&window, 0, 10);
        assert(Encoder_WindowStep(&window, 0, 10) == 0);
        Encoder_WindowReset(&window);
        for (unsigned i = 0; i < 100; ++i) {
            float speed = Encoder_WindowStep(&window, i % 20 < 7 ? 5 : 0, 20);
            if (i >= 19) assert(speed == 1.75f);
        }
        Encoder_WindowReset(&window);
        for (unsigned i = 0; i < 50; ++i) Encoder_WindowStep(&window, 2, 25);
        assert(Encoder_WindowStep(&window, 2, 10) == 2);
        assert(Encoder_WindowStep(&window, 2, 20) == 2);
        assert(Encoder_WindowStep(&window, 2, 0) == 2);
        assert(Encoder_WindowStep(&window, 2, 100) == 2);
        assert(Wheel_RPMWindowTicks(20) == 10);
        assert(Wheel_RPMWindowTicks(10) == 20);
        assert(Wheel_RPMWindowTicks(8) == 25);
        assert(Wheel_RPMWindowTicks(25) == 8);
        assert(Wheel_RPMWindowTicks(0) == 25);
        assert(Wheel_RPMWindowTicks(NAN) == 25);
    }
    for (int delta = -32768; delta < 32768; ++delta) {
        uint16_t previous = 60000u;
        assert(Encoder_Delta16((uint16_t)(previous + delta), previous) == delta);
    }
    for (int target = -25; target <= 25; ++target) {
        filtered = 0.0f;
        for (unsigned i = 0; i < 500u; ++i) filtered = Encoder_Filter(filtered, target);
        assert(fabsf(filtered - (float)target) < 0.0001f);
    }
    pid.kp = 2.0f; pid.ki = .3f; pid.max_output = 100.0f;
    for (unsigned i = 0; i < 100u; ++i) {
        float before = pid.output;
        PID_Incremental_Calc(&pid, 25.0f, 0.0f);
        assert(pid.output <= 100.0f && pid.output - before <= 30.0f);
    }
    PID_Reset(&pid);
    assert(pid.output == 0.0f && pid.err_last == 0.0f && pid.kp == 2.0f);
}

int main(int argc, char **argv)
{
    assert(argc == 2 || argc == 3);
    if (!strcmp(argv[1], "math")) { test_math(); puts("math: PASS"); return 0; }
    Autotune_SelectBootMode();
    assert(Autotune_IsEnabled());
    Autotune_Init();
    exchange("HELLO\n", "HELLO ELC_AUTOTUNE_V1 INC_PI_COUNTS_10MS 10 50 5500\n");
    exchange("INFO\n", "INFO ELC_INC_PI_V1 ");
    if (!strncmp(argv[1], "hold15", 6)) {
        exchange("LONGINFO\n", "LONGINFO ELC_HOLD15_V1 150 16500 30");
        arm();
        if (!strcmp(argv[1], "hold15_validation")) {
            exchange("HOLDRPM15 7 2 .3 2 .3 30 151\n", "ERR LONG_PROFILE");
            exchange("HOLDRPM15 7 2 .3 2 .3 31 100\n", "ERR LONG_PROFILE");
            assert(configured == 0);
        } else {
            clear_output(); feed("HOLDRPM15 7 2 .3 2 .3 30 100\n");
            for (unsigned i = 0; i <= 1650; ++i) {
                if (i % 10 == 0 && strcmp(argv[1], "hold15_heartbeat")) feed("PING\n");
                if (i == 600 && !strcmp(argv[1], "hold15_stop")) feed("STOP\n");
                tick_pair(i ? 3 : 0, i ? 4 : 0); Autotune_Poll();
                if (i == 0) assert(strstr(output, "HOLD15ACK 7"));
                if (!strcmp(argv[1], "hold15")) {
                    if (i >= 50 && i < 1550) {
                        assert(fabsf(measured.target_l - 5.4f) < .00001f);
                        assert(fabsf(measured.target_r - 6.6f) < .00001f);
                    } else assert(measured.pwm_l == 0 && measured.pwm_r == 0);
                }
            }
            assert(measured.pwm_l == 0 && measured.pwm_r == 0);
            if (!strcmp(argv[1], "hold15_heartbeat")) exchange("STATUS\n", "STATUS 7 4 10");
            else if (!strcmp(argv[1], "hold15_stop")) exchange("STATUS\n", "STATUS 7 3 0");
            else {
                exchange("STATUS\n", "STATUS 7 2 0 111 16500 0");
                exchange("DUMP\n", "BEGIN 7 111\n0,");
                assert(strstr(output, "\n16500,3,4,0.0000,0.0000"));
                tick(0); tick(0);
                exchange("RAWDUMP\n", "RAWBEGIN15 7 551 1 1 30.0000\n");
                char *line = strtok(output, "\n");
                for (unsigned i = 0; i <= 550; ++i) {
                    unsigned t, pl, pr, period; int l, r;
                    line = strtok(NULL, "\n"); assert(line);
                    assert(sscanf(line, "%u,%d,%d,%u,%u,%u", &t, &l, &r, &pl, &pr, &period) == 6);
                    assert(t == i * 30 && l == (i ? 9 : 0) && r == (i ? 12 : 0));
                    assert(period == (i ? 30000u : 10000u));
                    assert(pl == (t >= 500 && t < 15500 ? 100u : 0u) && pr == pl);
                }
                assert(!strcmp(strtok(NULL, "\n"), "RAWEND"));
            }
        }
    } else if (!strcmp(argv[1], "drive_profile") || !strcmp(argv[1], "drive_step") || !strcmp(argv[1], "drive_cancel")) {
        int hold = strcmp(argv[1], "drive_step") != 0;
        char info[128];
        snprintf(info, sizeof info,
                 "DRIVEINFO ELC_DRIVE_RPM_V1 2.000000 1.000000 2.000000 1.000000 %.4f 150 1 1\n",
                 (double)CAR_DRIVE_MAX_RPM);
        exchange("DRIVEINFO\n", info);
        arm(); clear_output();
        feed(hold ? "DRIVERPM 7 2 1 2 1 20 150\n" : "DRIVESTEP 7 2 1 2 1 20 150\n");
        assert(configured == 0);
        if (!strcmp(argv[1], "drive_cancel")) {
            feed("STOP\n"); tick(0);
            assert(configured == 0 && measured.pwm_l == 0);
        } else {
            for (unsigned i = 0; i <= 550; ++i) {
                if (i % 10 == 0) feed("PING\n");
                tick(i ? 3 : 0); Autotune_Poll();
                if (i == 0) assert(strstr(output, hold ? "DRIVEACK 7" : "DRIVESTEPACK 7"));
                if (i == 50) assert(fabsf(measured.target_l - (hold ? 3.6f : 1.44f)) < .00001f);
                if (i == 150) assert(fabsf(measured.target_r - 4.4f) < .00001f);
                if (i == 450) assert(measured.pwm_l == 0 && measured.pwm_r == 0);
            }
            exchange("STATUS\n", "STATUS 7 2 0 111 5500 0");
            assert(configured == 1);
        }
    } else if (!strcmp(argv[1], "drive_validation")) {
        char over[64];
        arm();
        exchange("DRIVERPM 7 2 .8 2 1 20 150\n", "ERR DRIVE_PROFILE");
        /* One RPM past the configured drive ceiling must be refused. */
        snprintf(over, sizeof over, "DRIVERPM 7 2 1 2 1 %u 150\n",
                 (unsigned)(CAR_DRIVE_MAX_RPM + 1.0f));
        exchange(over, "ERR DRIVE_PROFILE");
        exchange("DRIVESTEP 7 2 1 2 1 20 100\n", "ERR DRIVE_PROFILE");
        assert(configured == 0 && measured.pwm_l == 0);
    } else if (!strcmp(argv[1], "rpm_profile") || !strcmp(argv[1], "rpm_hold") || !strcmp(argv[1], "rpm_hold30") || !strcmp(argv[1], "rpm_hold90") || !strcmp(argv[1], "rpm_hold100") || !strcmp(argv[1], "rpm_hold130") || !strcmp(argv[1], "raw_trace")) {
        int p130 = !strcmp(argv[1], "rpm_hold130"), p100 = !strcmp(argv[1], "rpm_hold100"), p90 = !strcmp(argv[1], "rpm_hold90");
        int raw_mode = !strcmp(argv[1], "raw_trace") || p90 || p100 || p130;
        int hold = strcmp(argv[1], "rpm_profile") != 0;
        float peak = p130 ? 130.0f : p100 ? 100.0f : p90 ? 90.0f : (!strcmp(argv[1], "rpm_hold30") || raw_mode) ? 30.0f : 20.0f;
        float scale = peak / 20.0f;
        char request[128], expected_ack[160], expected_raw[100];
        if (p90 || p100 || p130) pwm_cap = p130 ? 450u : p100 ? 350u : 300u;
        exchange("CAL\n", "CAL MANUAL_REF_V1 1080 1320\n");
        exchange("RPMINFO\n", "RPMINFO ELC_RPM_PI_ADAPTIVE_V1\n");
        snprintf(request, sizeof request, "%s 7 2 .3 2 .3 %.0f %u\n", hold ? "HOLDRPM" : "RUNRPM", peak, pwm_cap);
        snprintf(expected_ack, sizeof expected_ack, "%s 7 2.000000 0.300000 2.000000 0.300000 %.4f %u", hold ? "HOLDACK" : "RPMACK", peak, pwm_cap);
        arm(); clear_output(); feed(request);
        assert(!strstr(output, "RPMACK"));
        for (unsigned i = 0; i <= 550; ++i) {
            if (i % 10 == 0) feed("PING\n");
            tick(i ? 3 : 0); Autotune_Poll();
            if (i == 0) assert(strstr(output, expected_ack));
            if (i == 50) {
                assert(fabsf(measured.target_l - scale * (hold ? 3.6f : 1.44f)) < .00001f);
                assert(fabsf(measured.target_r - scale * (hold ? 4.4f : 1.76f)) < .00001f);
            }
            if (i == 150) {
                assert(fabsf(measured.target_l - scale * 3.6f) < .00001f);
                assert(fabsf(measured.target_r - scale * 4.4f) < .00001f);
            }
            if (i == 300) {
                assert(fabsf(measured.target_l - scale * (hold ? 3.6f : 1.8f)) < .00001f);
                assert(fabsf(measured.target_r - scale * (hold ? 4.4f : 2.2f)) < .00001f);
            }
            if (i == 450) assert(measured.pwm_l == 0 && measured.pwm_r == 0);
            if (raw_mode && i == 200) exchange("RAWDUMP\n", "ERR BUSY");
        }
        exchange("STATUS\n", "STATUS 7 2 0 111 5500 0");
        assert(measured.pwm_l == 0 && measured.pwm_r == 0);
        if (raw_mode) {
            tick(0); tick(0); /* DONE must not append duplicate final samples. */
            exchange("RAWINFO\n", "RAWINFO ELC_RAW10_V1 10 552");
            snprintf(expected_raw, sizeof expected_raw, "RAWBEGIN 7 551 1 1 %.4f\n", peak);
            exchange("RAWDUMP\n", expected_raw);
            char *line = strtok(output, "\n");
            for (unsigned i = 0; i <= 550; ++i) {
                unsigned t, pl, pr, period;
                int l, r;
                line = strtok(NULL, "\n"); assert(line);
                assert(sscanf(line, "%u,%d,%d,%u,%u,%u", &t, &l, &r, &pl, &pr, &period) == 6);
                assert(t == i * 10 && l == (i ? 3 : 0) && r == l && period == 10000);
                assert(pl == (i >= 50 && i < 450 ? pwm_cap : 0u) && pr == pl);
            }
            assert(!strcmp(strtok(NULL, "\n"), "RAWEND"));
        }
    } else if (!strcmp(argv[1], "rpm_validation")) {
        arm();
        exchange("RUNRPM 7 2 .3 2 .3 5 100\n", "ERR RPM");
        exchange("RUNRPM 7 2 .3 2 .3 nan 100\n", "ERR GAINS");
        exchange("RUNRPM 7 2 .3 2 .3 131 100\n", "ERR GAINS");
        exchange("RUN 7 2 .3 2 .3 26 100\n", "ERR GAINS");
        /* One RPM past the configured drive ceiling must be refused. */
        char over[64];
        snprintf(over, sizeof over, "DRIVERPM 7 2 1 2 1 %u 150\n",
                 (unsigned)(CAR_DRIVE_MAX_RPM + 1.0f));
        exchange(over, "ERR DRIVE_PROFILE");
        exchange("RUNRPM 7 2 .3 2 .3 20 451\n", "ERR GAINS");
        exchange("HOLDRPM 7 2 .3 2 .3 5 100\n", "ERR RPM");
        assert(configured == 0 && measured.pwm_l == 0);
    } else if (!strcmp(argv[1], "jog_raw")) {
        arm();
        clear_output(); feed("JOG 1 111 0 800\n"); tick_pair(0, 0); Autotune_Poll();
        assert(strstr(output, "JOGACK 1 111 0 800"));
        exchange("RAWDUMP\n", "ERR BUSY");
        for (unsigned i = 0; i < 80; ++i) tick_pair(3, 0);
        assert(measured.pwm_l == 0 && measured.pwm_r == 0);
        exchange("JOGRAWINFO\n", "JOGRAWINFO ELC_JOG_RAW10_V1 10 800");
        exchange("RAWDUMP\n", "RAWJOG 1 81 111 0 800\n");
        char *line = strtok(output, "\n");
        for (unsigned i = 0; i <= 80; ++i) {
            unsigned t, pl, pr, period;
            int l, r;
            line = strtok(NULL, "\n"); assert(line);
            assert(sscanf(line, "%u,%d,%d,%u,%u,%u", &t, &l, &r, &pl, &pr, &period) == 6);
            assert(t == i * 10 && l == (i ? 3 : 0) && r == 0 && period == 10000);
            assert(pl == (i < 80 ? 111u : 0u) && pr == 0);
        }
        assert(!strcmp(strtok(NULL, "\n"), "RAWEND"));
    } else if (!strncmp(argv[1], "jog_", 4)) {
        exchange("JOG 9 150 0 800\n", "ERR NOT_ARMED");
        arm();
        if (!strcmp(argv[1], "jog_validation")) {
            const char *invalid[] = {"JOG 9 151 0 800\n", "JOG 9 150 0 801\n",
                "JOG 9 0 0 500\n", "JOG 9 100 100 500\n", "JOG 9 nan 0 500\n",
                "JOG 9 100.5 0 500\n", "JOG 9 100 0 49\n", "JOG 9 100 0 500.5\n",
                "JOG 9 100 0 51\n"};
            for (unsigned i = 0; i < sizeof invalid / sizeof invalid[0]; ++i)
                exchange(invalid[i], "ERR JOG");
            assert(measured.pwm_l == 0 && measured.pwm_r == 0);
        } else if (!strcmp(argv[1], "jog_cancel")) {
            feed("JOG 9 150 0 800\nSTOP\n"); tick(0);
            assert(measured.pwm_l == 0 && measured.pwm_r == 0);
            exchange("STATUS\n", "STATUS 0 3 0");
        } else if (!strcmp(argv[1], "jog_expiry")) {
            for (unsigned i = 0; i < 499; ++i) tick(0);
            feed("JOG 9 150 0 800\n"); tick(0); Autotune_Poll();
            assert(strstr(output, "ERR ARM_EXPIRED"));
            assert(measured.pwm_l == 0 && measured.pwm_r == 0);
        } else {
            clear_output(); feed("JOG 9 -150 0 800\n");
            assert(!strstr(output, "JOGACK"));
            tick(0); Autotune_Poll();
            assert(strstr(output, "JOGACK 9 -150 0 800\n"));
            assert(configured == 0 && measured.pwm_l == -150 && measured.pwm_r == 0);
            exchange("JOG 10 0 100 800\n", "ERR NOT_ARMED");
            if (!strcmp(argv[1], "jog_complete")) {
                for (unsigned i = 0; i < 79; ++i) tick(-3);
                assert(measured.pwm_l == -150);
                tick(-3); exchange("STATUS\n", "STATUS 9 2 0 17 800 0");
                exchange("RAWDUMP\n", "ERR RAW_MODE");
            } else if (!strcmp(argv[1], "jog_key")) {
                stop_key = 1; tick(0); exchange("STATUS\n", "STATUS 9 3 0");
            } else if (!strcmp(argv[1], "jog_uart")) {
                Autotune_OnUartError(); Autotune_Poll();
                exchange("\nSTATUS\n", "STATUS 9 4 11");
            } else if (!strcmp(argv[1], "jog_speed")) {
                tick(101); exchange("STATUS\n", "STATUS 9 4 14");
            } else if (!strcmp(argv[1], "jog_control_fault")) {
                Control_LatchFault(CONTROL_FAULT_TIMING); tick(0);
                exchange("STATUS\n", "STATUS 9 4 1");
            } else if (!strcmp(argv[1], "jog_stop")) {
                exchange("STOP\n", "STOPPED"); tick(0);
            } else assert(0);
            assert(measured.pwm_l == 0 && measured.pwm_r == 0);
        }
    } else if (!strcmp(argv[1], "validation")) {
        exchange("RUN 7 2 .3 2 .3 5 100\n", "ERR NOT_ARMED");
        arm();
        exchange("RUN 7 nan .3 2 .3 5 100\n", "ERR GAINS");
        exchange("RUN 7 2 .3 2 .3 5 201\n", "ERR GAINS");
        exchange("RUN 7 2 .3 2 .3 5 100.5\n", "ERR GAINS");
        exchange("RUN 4294967296 2 .3 2 .3 5 100\n", "ERR GAINS");
        exchange("RUN 7 2 .3 2 .3 5 100 junk\n", "ERR COMMAND");
        assert(configured == 0 && measured.pwm_l == 0.0f);
    } else if (!strcmp(argv[1], "slow_motion")) {
        for (unsigned i = 0; i < 40u; ++i) tick(1);
        exchange("ARM 1 -1\n", "ERR NOT_STOPPED");
        assert(configured == 0);
    } else if (!strcmp(argv[1], "arm_expiry")) {
        arm();
        for (unsigned i = 0; i < 501u; ++i) tick(0);
        exchange("RUN 7 2 .3 2 .3 5 100\n", "ERR NOT_ARMED");
        assert(configured == 0 && measured.pwm_l == 0.0f);
    } else if (!strcmp(argv[1], "cancel_pending")) {
        arm(); feed("RUN 7 2 .3 2 .3 5 100\nSTOP\n");
        tick(0); assert(configured == 0);
        exchange("STATUS\n", "STATUS 0 3 0 0 0 0 1");
    } else if (!strcmp(argv[1], "pending_expiry")) {
        arm();
        for (unsigned i = 0; i < 499u; ++i) tick(0);
        feed("RUN 7 2 .3 2 .3 5 100\n");
        clear_output(); tick(0); Autotune_Poll();
        assert(strstr(output, "ERR ARM_EXPIRED"));
        assert(configured == 0 && measured.pwm_l == 0.0f);
    } else if (!strcmp(argv[1], "complete")) {
        start();
        for (unsigned i = 0; i < 550u; ++i) {
            if (i % 10u == 0u) feed("PING\n");
            tick(3);
        }
        exchange("STATUS\n", "STATUS 7 2 0 111 5500 0");
        assert(measured.pwm_l == 0.0f && measured.pwm_r == 0.0f);
        exchange("DUMP\n", "BEGIN 7 111\n0,0,0,0.0000,0.0000");
        assert(strstr(output, "5500,3,3,0.0000,0.0000"));
        assert(strstr(output, "\nEND\n"));
        if (argc == 3 && !strcmp(argv[2], "--dump")) {
            fputs(output, stdout);
            return 0;
        }
    } else if (!strcmp(argv[1], "raw_period_overflow")) {
        start(); now += 10u; Autotune_BeforeControl(now);
        measured.period_us = 70000u;
        Autotune_AfterControl(&measured);
        exchange("STATUS\n", "STATUS 7 4 15");
        assert(measured.pwm_l == 0 && measured.pwm_r == 0);
    } else if (!strcmp(argv[1], "heartbeat")) {
        start();
        for (unsigned i = 0; i < 100u; ++i) tick(3);
        exchange("STATUS\n", "STATUS 7 4 10");
        assert(measured.pwm_l == 0.0f);
    } else if (!strcmp(argv[1], "direction")) {
        start();
        for (unsigned i = 0; i < 50u; ++i) tick(0);
        feed("PING\n");
        for (unsigned i = 0; i < 3u; ++i) tick(-3);
        exchange("STATUS\n", "STATUS 7 4 13");
        assert(measured.pwm_l == 0.0f);
    } else if (!strncmp(argv[1], "low_stall", 9) || !strcmp(argv[1], "no_motion")) {
        pwm_cap = !strcmp(argv[1], "no_motion") ? 50u : (unsigned)atoi(argv[1] + 9);
        if (!strcmp(argv[1], "no_motion")) pwm_fraction = 0.0f;
        start();
        for (unsigned i = 0; i < 160u; ++i) {
            if (i % 10u == 0u) feed("PING\n");
            tick(0);
        }
        exchange("STATUS\n", "STATUS 7 4 12");
        assert(measured.pwm_l == 0.0f);
    } else if (!strcmp(argv[1], "sparse_motion")) {
        pwm_cap = 20u; start();
        for (unsigned i = 0; i < 550u; ++i) {
            if (i % 10u == 0u) feed("PING\n");
            tick(i % 4u == 0u ? 1 : 0);
        }
        exchange("STATUS\n", "STATUS 7 2 0 111 5500 0");
    } else if (!strcmp(argv[1], "stall")) {
        start();
        for (unsigned i = 0; i < 110u; ++i) {
            if (i % 10u == 0u) feed("PING\n");
            tick(0);
        }
        exchange("STATUS\n", "STATUS 7 4 12");
        assert(measured.pwm_l == 0.0f);
    } else if (!strcmp(argv[1], "stop_key")) {
        start(); stop_key = 1; tick(0);
        exchange("STATUS\n", "STATUS 7 3 0");
        assert(measured.pwm_l == 0.0f);
    } else if (!strcmp(argv[1], "uart_fault")) {
        start(); Autotune_OnUartError();
        assert(Control_GetFault() == CONTROL_FAULT_UART && measured.pwm_l == 0.0f);
        Autotune_Poll(); exchange("\nSTATUS\n", "STATUS 7 4 11");
    } else if (!strcmp(argv[1], "frame")) {
        char large[203];
        start(); memset(large, 'x', 200u); large[200] = '\n'; large[201] = 0;
        clear_output(); feed(large); Autotune_Poll();
        assert(measured.pwm_l == 0.0f);
        exchange("STATUS\n", "STATUS 7 4 16");
    } else if (!strcmp(argv[1], "overflow")) {
        start();
        for (unsigned i = 0; i < 300u; ++i) Autotune_OnByte('X');
        assert(Control_GetFault() == CONTROL_FAULT_UART);
        assert(measured.pwm_l == 0.0f);
    } else assert(0);
    printf("%s: PASS\n", argv[1]);
    return 0;
}
