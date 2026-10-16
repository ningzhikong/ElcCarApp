#include "autotune.h"
#include "control_math.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int huart1;
static uint32_t now, mask, control_fault;
static ControlView measured;
static int stop_key, configured;
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
void Control_ConfigurePI(float pl, float il, float pr, float ir, float limit)
{
    assert(pl == 2.0f && il == 0.3f && pr == 2.0f && ir == 0.3f && limit == 100.0f);
    ++configured;
    Control_Stop();
}
void Control_SetTargets(float l, float r)
{
    measured.target_l = l; measured.target_r = r;
    measured.pwm_l = l > 0.0f ? 100.0f : 0.0f;
    measured.pwm_r = r > 0.0f ? 100.0f : 0.0f;
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
static void tick(int raw)
{
    now += 10u;
    measured.tick_ms = now;
    measured.period_us = 10000u;
    measured.raw_l = measured.raw_r = raw;
    measured.speed_l = measured.speed_r = (float)raw;
    Autotune_BeforeControl(now);
    Autotune_AfterControl(&measured);
}
static void arm(void)
{
    for (unsigned i = 0u; i < 35u; ++i) tick(0);
    exchange("ARM 1 -1\n", "ARMED 1 -1\n");
}
static void start(void)
{
    arm(); clear_output();
    feed("RUN 7 2 .3 2 .3 5 100\n");
    assert(!strstr(output, "ACK"));
    assert(configured == 0);
    tick(0); Autotune_Poll();
    assert(configured == 1);
    assert(strstr(output, "ACK 7 2.000000 0.300000 2.000000 0.300000 5.0000 100\n"));
}

static void test_math(void)
{
    float filtered;
    Speed_PID_t pid;
    memset(&pid, 0, sizeof pid);
    assert(Encoder_Delta16(65531u, 0u) == -5);
    assert(Encoder_Delta16(2u, 65534u) == 4);
    assert(Encoder_Delta16(65534u, 2u) == -4);
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
    if (!strcmp(argv[1], "validation")) {
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
