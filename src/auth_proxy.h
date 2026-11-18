#pragma once

#include <stdint.h>

/*
 * Shared-bus authentication relay for A1 printer firmware with the
 * device-unbound 0x040E challenge-response behaviour.
 *
 * The feature is opt-in so existing firmware variants keep their current
 * behaviour. Enable it from a dedicated PlatformIO environment.
 */
#ifndef BMCU_AUTH_PROXY_ENABLE
#define BMCU_AUTH_PROXY_ENABLE 0
#endif

/* The AMS index of the official AMS that will produce the signature. */
#ifndef BMCU_AUTH_PROXY_TARGET_AMS_NUM
#define BMCU_AUTH_PROXY_TARGET_AMS_NUM 0
#endif

/* Bus address of the device that signs the relayed challenge. */
#ifndef BMCU_AUTH_PROXY_TARGET_BUS_ADDRESS
#define BMCU_AUTH_PROXY_TARGET_BUS_ADDRESS 0x0700u
#endif

/* Only accept challenges whose selector byte belongs to this BMCU. */
#ifndef BMCU_AUTH_PROXY_FILTER_LOCAL_INDEX
#define BMCU_AUTH_PROXY_FILTER_LOCAL_INDEX 0
#endif

/* Payload byte used by the tested firmware for the AMS selector. */
#ifndef BMCU_AUTH_PROXY_DEVICE_INDEX_OFFSET
#define BMCU_AUTH_PROXY_DEVICE_INDEX_OFFSET 4
#endif

/* Rewrite that selector to the official AMS index before retransmission. */
#ifndef BMCU_AUTH_PROXY_REWRITE_DEVICE_INDEX
#define BMCU_AUTH_PROXY_REWRITE_DEVICE_INDEX 0
#endif

/* Bump the packet number to make the reissued request a fresh transaction. */
#ifndef BMCU_AUTH_PROXY_BUMP_PACKET_NUMBER
#define BMCU_AUTH_PROXY_BUMP_PACKET_NUMBER 0
#endif

/*
 * The signed response comprises four 64-byte fragments. The printer is also
 * receiving the original AMS frames; this timeout only controls how long BMCU
 * pauses its own ordinary transmissions while harvesting the chunks.
 */
#ifndef BMCU_AUTH_PROXY_TIMEOUT_MS
#define BMCU_AUTH_PROXY_TIMEOUT_MS 30
#endif

/*
 * Give an official AMS a short chance to answer the original challenge before
 * BMCU retransmits it. This avoids duplicate responses and needless RS485
 * contention when the AMS already accepted the printer's frame.
 */
#ifndef BMCU_AUTH_PROXY_DIRECT_RESPONSE_GRACE_MS
#define BMCU_AUTH_PROXY_DIRECT_RESPONSE_GRACE_MS 5
#endif

#ifndef BMCU_AUTH_PROXY_RETRY_COUNT
#define BMCU_AUTH_PROXY_RETRY_COUNT 1
#endif

#ifndef BMCU_AUTH_PROXY_RX_QUEUE_DEPTH
#define BMCU_AUTH_PROXY_RX_QUEUE_DEPTH 6
#endif

#ifndef BMCU_AUTH_PROXY_FRAME_MAX
#define BMCU_AUTH_PROXY_FRAME_MAX 192
#endif

struct AuthProxyStats
{
    uint32_t challenges_seen;
    uint32_t forwards_sent;
    uint32_t response_frames;
    uint32_t completed;
    uint32_t timeouts;
    uint32_t wrong_index;
    uint32_t queue_drops;
    uint32_t invalid_frames;
};

#ifdef __cplusplus
extern "C" {
#endif

void auth_proxy_init(void);
void auth_proxy_run(void);

/* True while BMCU must not send ordinary status/heartbeat responses. */
uint8_t auth_proxy_tx_locked(void);

/* Bus-completion callbacks installed by auth_proxy_init(). */
uint8_t auth_proxy_filter_frame(const uint8_t *data, uint16_t length);
void auth_proxy_capture_frame(const uint8_t *data, uint16_t length);

void auth_proxy_get_stats(AuthProxyStats *stats);

#ifdef __cplusplus
}
#endif
