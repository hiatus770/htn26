#include "kart_net.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#define KART_PORT 5005
#define KART_MAGIC 0x4B415254u // 'KART'

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t badge_id[3];
    float x, y, heading;
    uint8_t lap;
    uint8_t finished;
} kart_packet_t;

static const char *TAG = "kart_net";
static int s_sock = -1;
static uint8_t s_my_id[3];

void kart_net_init(void)
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
        .sin_port = htons(KART_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(s_sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed: errno %d", errno);
        close(s_sock);
        s_sock = -1;
        return;
    }

    // Non-blocking so kart_net_poll() never stalls the game loop.
    int flags = fcntl(s_sock, F_GETFL, 0);
    fcntl(s_sock, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI(TAG, "kart UDP ready on port %d, my_id=%02x%02x%02x",
             KART_PORT, s_my_id[0], s_my_id[1], s_my_id[2]);
}

void kart_net_send(float x, float y, float heading, uint8_t lap, bool finished)
{
    if (s_sock < 0) return;

    kart_packet_t pkt = {
        .magic = KART_MAGIC,
        .x = x, .y = y, .heading = heading,
        .lap = lap,
        .finished = finished ? 1 : 0,
    };
    memcpy(pkt.badge_id, s_my_id, 3);

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port = htons(KART_PORT),
        .sin_addr.s_addr = htonl(INADDR_BROADCAST),
    };
    sendto(s_sock, &pkt, sizeof(pkt), 0, (struct sockaddr *)&dest, sizeof(dest));
}

void kart_net_poll(kart_peer_t *peer)
{
    if (s_sock < 0) return;

    for (;;) {
        kart_packet_t pkt;
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        int n = recvfrom(s_sock, &pkt, sizeof(pkt), 0, (struct sockaddr *)&from, &from_len);
        if (n != (int)sizeof(pkt)) break; // EWOULDBLOCK or a malformed/partial packet
        if (pkt.magic != KART_MAGIC) continue;
        if (memcmp(pkt.badge_id, s_my_id, 3) == 0) continue; // our own broadcast

        peer->valid = true;
        peer->x = pkt.x;
        peer->y = pkt.y;
        peer->heading = pkt.heading;
        peer->lap = pkt.lap;
        peer->finished = pkt.finished != 0;
        peer->last_seen_ms = (uint32_t)(esp_timer_get_time() / 1000);
    }
}
