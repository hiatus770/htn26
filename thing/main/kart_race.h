#pragma once
#include "hal_buttons.h"

void kart_race_init(void);
void kart_race_reset(void);
void kart_race_step(const button_state_t *btn);
