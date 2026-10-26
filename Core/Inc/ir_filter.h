#ifndef CAR_IR_FILTER_H
#define CAR_IR_FILTER_H
#include <stdint.h>
#include <string.h>

/* Three consecutive samples per channel; no averaging of digital colors. */
typedef struct {
    uint8_t levels, initialized, previous_raw, seen;
    uint8_t candidate[8], streak[8], flips[8], noisy;
    uint32_t window_tick, last_change_tick, last_tick;
} IRFilter;

static inline void IRFilter_Reset(IRFilter *f)
{ memset(f, 0, sizeof *f); }

static inline void IRFilter_Step(IRFilter *f, uint8_t raw, uint32_t now)
{
    if (f->seen && now - f->last_tick > 40u) IRFilter_Reset(f);
    if (!f->seen) {
        f->seen = 1u; f->previous_raw = raw;
        f->window_tick = f->last_change_tick = now;
    }
    if (now - f->window_tick >= 200u) {
        memset(f->flips, 0, sizeof f->flips);
        f->window_tick = now;
    }
    uint8_t changes = raw ^ f->previous_raw;
    if (changes) f->last_change_tick = now;
    for (unsigned i = 0; i < 8; ++i) {
        uint8_t bit = (uint8_t)(1u << i), value = (raw & bit) != 0u;
        if (changes & bit) {
            if (f->flips[i] < 4u) ++f->flips[i];
            if (f->flips[i] >= 4u) f->noisy = 1u;
        }
        if (!f->streak[i] || value != f->candidate[i]) {
            f->candidate[i] = value; f->streak[i] = 1u;
        } else if (f->streak[i] < 3u) ++f->streak[i];
        if (f->streak[i] == 3u) {
            f->levels = value ? f->levels | bit : f->levels & (uint8_t)~bit;
            f->initialized |= bit;
        }
    }
    if (now - f->last_change_tick >= 200u) f->noisy = 0u;
    f->previous_raw = raw; f->last_tick = now;
}
#endif
