#include "duel_net.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#define DUEL_PORT 5006
#define DUEL_MAGIC 0x4455454Cu // 'DUEL'

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t badge_id[3];
    float x, y, dir_x, dir_y;
    int16_t hp;
    uint8_t alive;
    uint32_t shot_seq;
} duel_packet_t;

static const char *TAG = "duel_net";
static int s_sock = -1;
static uint8_t s_my_id[3];
static uint32_t s_last_peer_shot_seq = 0;
static bool s_have_seen_peer = false;

void duel_net_init(void)
{
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    memcpy(s_my_id, mac + 3, 3);

    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno %d", errno);
        return;
    }

    int broadcast_en = 1;
    setsockopt(s_sock, SOL_SOCKET, SO_BROADCAST, &broadcast_en, sizeof(broadcast_en));

    struct sockaddr_in local_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(DUEL_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(s_sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed: errno %d", errno);
        close(s_sock);
        s_sock = -1;
        return;
    }

    int flags = fcntl(s_sock, F_GETFL, 0);
    fcntl(s_sock, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI(TAG, "duel UDP ready on port %d, my_id=%02x%02x%02x",
             DUEL_PORT, s_my_id[0], s_my_id[1], s_my_id[2]);
}

void duel_net_get_my_id(uint8_t out[3])
{
    memcpy(out, s_my_id, 3);
}

void duel_net_send(float x, float y, float dir_x, float dir_y, int16_t hp, bool alive, uint32_t shot_seq)
{
    if (s_sock < 0) return;

    duel_packet_t pkt = {
        .magic = DUEL_MAGIC,
        .x = x, .y = y, .dir_x = dir_x, .dir_y = dir_y,
        .hp = hp,
        .alive = alive ? 1 : 0,
        .shot_seq = shot_seq,
    };
    memcpy(pkt.badge_id, s_my_id, 3);

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port = htons(DUEL_PORT),
        .sin_addr.s_addr = htonl(INADDR_BROADCAST),
    };
    sendto(s_sock, &pkt, sizeof(pkt), 0, (struct sockaddr *)&dest, sizeof(dest));
}

void duel_net_poll(duel_peer_t *peer, bool *peer_fired)
{
    *peer_fired = false;
    if (s_sock < 0) return;

    for (;;) {
        duel_packet_t pkt;
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        int n = recvfrom(s_sock, &pkt, sizeof(pkt), 0, (struct sockaddr *)&from, &from_len);
        if (n != (int)sizeof(pkt)) break;
        if (pkt.magic != DUEL_MAGIC) continue;
        if (memcmp(pkt.badge_id, s_my_id, 3) == 0) continue;

        peer->valid = true;
        memcpy(peer->badge_id, pkt.badge_id, 3);
        peer->x = pkt.x;
        peer->y = pkt.y;
        peer->dir_x = pkt.dir_x;
        peer->dir_y = pkt.dir_y;
        peer->hp = pkt.hp;
        peer->alive = pkt.alive != 0;
        peer->last_seen_ms = (uint32_t)(esp_timer_get_time() / 1000);

        if (s_have_seen_peer && pkt.shot_seq != s_last_peer_shot_seq) {
            *peer_fired = true;
        }
        s_last_peer_shot_seq = pkt.shot_seq;
        s_have_seen_peer = true;
    }
}
