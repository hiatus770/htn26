#include "game.h"
#include "hal_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    hal_wifi_init();
    game_init();
    while (1) {
        game_step();
        vTaskDelay(pdMS_TO_TICKS(20)); // targets ~20-30 fps alongside render time
    }
}
