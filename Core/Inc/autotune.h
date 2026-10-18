#ifndef CAR_AUTOTUNE_H
#define CAR_AUTOTUNE_H
#include "control.h"

/* Boot choice is latched until reset. No serial command can enter test mode. */
void Autotune_SelectBootMode(void);
int Autotune_IsEnabled(void);
void Autotune_Init(void);
void Autotune_Poll(void);
void Autotune_OnByte(uint8_t byte);
void Autotune_OnUartError(void);
void Autotune_BeforeControl(uint32_t now_ms);
void Autotune_AfterControl(const ControlView *view);
#endif
