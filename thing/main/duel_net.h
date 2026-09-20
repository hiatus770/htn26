#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool valid;
    uint8_t badge_id[3];
    float x, y;
    float dir_x, dir_y;
    int16_t hp;
    bool alive;
    uint32_t last_seen_ms;
} duel_peer_t;

void duel_net_init(void);

// Our own sender id (last 3 bytes of the station MAC), for deterministically
// picking which of two spawn points each badge uses.
void duel_net_get_my_id(uint8_t out[3]);

// Broadcasts our current state. shot_seq should increment by one each time
// we fire; the peer detects a new shot by seeing this value change.
void duel_net_send(float x, float y, float dir_x, float dir_y, int16_t hp, bool alive, uint32_t shot_seq);

// Drains incoming packets, updates *peer with the latest state, and sets
// *peer_fired true if the peer's shot_seq advanced since we last heard from
// them. On a fire event, peer->x/y/dir_x/dir_y is the shot's origin/aim --
// each badge is authoritative over whether ITS OWN current position was hit,
// so hit-testing uses live local position against this network-reported aim.
void duel_net_poll(duel_peer_t *peer, bool *peer_fired);
