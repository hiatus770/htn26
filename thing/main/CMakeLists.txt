idf_component_register(
    SRCS
        "app_main.c"
        "hal_buttons.c"
        "hal_display.c"
        "hal_led.c"
        "hal_wifi.c"
        "hal_accel.c"
        "kart_net.c"
        "kart_race.c"
        "duel_net.c"
        "duel.c"
        "sprite_pistol.c"
        "sprite_doomguy.c"
        "raycaster.c"
        "map.c"
        "game.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_lcd esp_wifi esp_netif esp_event nvs_flash lwip esp_timer esp_driver_i2c
)
