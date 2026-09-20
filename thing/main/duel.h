#pragma once
#include "hal_buttons.h"

void duel_init(void);
void duel_reset(void);
void duel_step(const button_state_t *btn);
