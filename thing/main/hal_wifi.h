#pragma once
#include <stdint.h>

typedef enum {
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_FAILED,
} wifi_state_t;

typedef struct {
    wifi_state_t state;
    char ip[16];   // valid once state == WIFI_STATE_CONNECTED
    int8_t rssi;   // dBm, valid once state == WIFI_STATE_CONNECTED
} wifi_status_t;

// Brings up NVS + station-mode WiFi and starts connecting in the
// background (event-driven; this call returns immediately).
void hal_wifi_init(void);

// Thread-safe snapshot of the current connection state.
void hal_wifi_get_status(wifi_status_t *out);
