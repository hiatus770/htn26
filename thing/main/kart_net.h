#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool valid; // true once we've received at least one packet from a peer
    float x, y;
    float heading;
    uint8_t lap;
    bool finished;
    uint32_t last_seen_ms;
} kart_peer_t;

// Opens the UDP broadcast socket used for badge-to-badge kart state. Must be
// called after WiFi is up (reads the station MAC as a lightweight sender id).
void kart_net_init(void);

// Broadcasts our current state on the local WiFi network. Caller throttles
// call frequency; this does not rate-limit internally.
void kart_net_send(float x, float y, float heading, uint8_t lap, bool finished);

// Drains any pending incoming packets, leaving *peer holding the most recent
// state received from another badge (ignores our own broadcasts).
void kart_net_poll(kart_peer_t *peer);
