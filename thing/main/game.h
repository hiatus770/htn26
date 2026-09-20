#pragma once

void game_init(void);
// One frame: poll input, update sim, render, blit to the panel, update LEDs.
// Call in a loop from app_main.
void game_step(void);
