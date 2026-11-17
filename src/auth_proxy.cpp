#include "auth_proxy.h"

#include "Flash_saves.h"
#include "_bus_hardware.h"
#include "crc_bus.h"
#include "hal/irq_wch.h"
#include "hal/time_hw.h"

#include <string.h>

#if BMCU_AUTH_PROXY_ENABLE

namespace {

constexpr uint8_t kLongFrameMinLength = 15u;
constexpr uint8_t kLongPayloadOffset = 13u;
constexpr uint8_t kFlagOffset = 1u;
constexpr uint8_t kPacketNumberOffset = 2u;
constexpr uint8_t kFrameLengthOffset = 4u;
constexpr uint8_t kHeaderCrcOffset = 6u;
constexpr uint8_t kTargetAddressOffset = 7u;
constexpr uint8_t kSourceAddressOffset = 9u;
constexpr uint8_t kCommandOffset = 11u;
constexpr uint16_t kAmsBusAddress = 0x0700u;
constexpr uint16_t kAuthChallengeCommand = 0x040Eu;
constexpr uint16_t kChallengePayloadLength = 37u;
constexpr uint16_t kResponsePayloadLength = 76u;
constexpr uint8_t kSignatureChunkLength = 64u;
constexpr uint8_t kSignatureChunkCount = 4u;

enum class ProxyState : uint8_t
{
    idle = 0,
    send_forward,
    wait_response,
};

struct QueuedFrame
{
    uint16_t length;
    uint8_t data[BMCU_AUTH_PROXY_FRAME_MAX];
};

static_assert(BMCU_AUTH_PROXY_RX_QUEUE_DEPTH >= 2, "auth proxy queue too small");
static_assert(BMCU_AUTH_PROXY_FRAME_MAX >= 91, "auth proxy frame buffer too small");
static_assert(BMCU_AUTH_PROXY_FRAME_MAX <= 255, "auth proxy queue index is uint8_t");
static_assert(BMCU_AUTH_PROXY_TARGET_AMS_NUM <= 3, "AMS index must be 0..3");
static_assert((uint8_t)BAMBU_BUS_AMS_NUM != (uint8_t)BMCU_AUTH_PROXY_TARGET_AMS_NUM,
              "BMCU and signing AMS must use different AMS indexes");

static QueuedFrame rx_queue[BMCU_AUTH_PROXY_RX_QUEUE_DEPTH];
static volatile uint8_t rx_head = 0u;
static volatile uint8_t rx_tail = 0u;

static uint8_t forward_frame[BMCU_AUTH_PROXY_FRAME_MAX];
static uint16_t forward_length = 0u;
static ProxyState state = ProxyState::idle;
static uint8_t tx_locked = 0u;
static uint8_t retries_left = 0u;
static uint8_t response_mask = 0u;
static uint32_t response_deadline = 0u;
static uint32_t send_deadline = 0u;
static uint16_t expected_packet_number = 0u;
static uint16_t expected_response_target = 0u;
static uint16_t expected_response_source = 0u;
static AuthProxyStats stats = {};

static inline uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8u);
}

static inline void write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)(value >> 8u);
}

static uint8_t queue_next(uint8_t value)
{
    value++;
    return (value >= BMCU_AUTH_PROXY_RX_QUEUE_DEPTH) ? 0u : value;
}

static bool valid_long_frame(const uint8_t *data, uint16_t length)
{
    if ((data == nullptr) || (length < kLongFrameMinLength) ||
        (length > BMCU_AUTH_PROXY_FRAME_MAX))
    {
        return false;
    }
    if ((data[0] != 0x3Du) || ((data[kFlagOffset] & 0x80u) != 0u))
    {
        return false;
    }
    if (read_u16(data + kFrameLengthOffset) != length)
    {
        return false;
    }
    if (data[kHeaderCrcOffset] != bus_crc8(data, kHeaderCrcOffset))
    {
        return false;
    }

    const uint16_t expected = bus_crc16(data, (uint32_t)length - 2u);
    return (data[length - 2u] == (uint8_t)(expected & 0xFFu)) &&
           (data[length - 1u] == (uint8_t)(expected >> 8u));
}

static void update_frame_crc(uint8_t *data, uint16_t length)
{
    data[kHeaderCrcOffset] = bus_crc8(data, kHeaderCrcOffset);
    const uint16_t crc = bus_crc16(data, (uint32_t)length - 2u);
    data[length - 2u] = (uint8_t)(crc & 0xFFu);
    data[length - 1u] = (uint8_t)(crc >> 8u);
}

static bool is_auth_command_frame(const uint8_t *data, uint16_t length)
{
    if ((data == nullptr) || (length < kLongFrameMinLength) ||
        (length > BMCU_AUTH_PROXY_FRAME_MAX))
    {
        return false;
    }
    if ((data[0] != 0x3Du) || ((data[kFlagOffset] & 0x80u) != 0u))
    {
        return false;
    }
    return read_u16(data + kCommandOffset) == kAuthChallengeCommand;
}

static bool is_challenge_frame(const uint8_t *data, uint16_t length)
{
    if (!valid_long_frame(data, length))
    {
        return false;
    }
    if (read_u16(data + kTargetAddressOffset) != kAmsBusAddress)
    {
        return false;
    }
    return ((uint16_t)length - kLongFrameMinLength) == kChallengePayloadLength;
}

static bool is_response_frame(const uint8_t *data, uint16_t length)
{
    if (!valid_long_frame(data, length))
    {
        return false;
    }
    return ((uint16_t)length - kLongFrameMinLength) == kResponsePayloadLength;
}

static void release_transaction(void)
{
    state = ProxyState::idle;
    tx_locked = 0u;
    retries_left = 0u;
    response_mask = 0u;
    response_deadline = 0u;
    send_deadline = 0u;
    expected_packet_number = 0u;
    expected_response_target = 0u;
    expected_response_source = 0u;
}

static void begin_transaction(const uint8_t *frame, uint16_t length)
{
    if (!is_challenge_frame(frame, length))
    {
        stats.invalid_frames++;
        return;
    }

    const uint8_t *payload = frame + kLongPayloadOffset;

#if BMCU_AUTH_PROXY_FILTER_LOCAL_INDEX
    if (payload[BMCU_AUTH_PROXY_DEVICE_INDEX_OFFSET] != (uint8_t)BAMBU_BUS_AMS_NUM)
    {
        stats.wrong_index++;
        return;
    }
#endif

    memcpy(forward_frame, frame, length);
    forward_length = length;

#if BMCU_AUTH_PROXY_REWRITE_DEVICE_INDEX
    forward_frame[kLongPayloadOffset + BMCU_AUTH_PROXY_DEVICE_INDEX_OFFSET] =
        (uint8_t)BMCU_AUTH_PROXY_TARGET_AMS_NUM;
#endif

#if BMCU_AUTH_PROXY_BUMP_PACKET_NUMBER
    write_u16(forward_frame + kPacketNumberOffset,
              (uint16_t)(read_u16(forward_frame + kPacketNumberOffset) + 1u));
#endif

    update_frame_crc(forward_frame, forward_length);

    stats.challenges_seen++;
    response_mask = 0u;
    retries_left = BMCU_AUTH_PROXY_RETRY_COUNT;
    expected_packet_number = read_u16(forward_frame + kPacketNumberOffset);
    expected_response_target = read_u16(forward_frame + kSourceAddressOffset);
    expected_response_source = read_u16(forward_frame + kTargetAddressOffset);
    state = ProxyState::send_forward;
    tx_locked = 1u;
    send_deadline = time_ticks32() + ms_to_ticks32(BMCU_AUTH_PROXY_TIMEOUT_MS);
}

static void note_response(const uint8_t *frame, uint16_t length)
{
    if (!is_response_frame(frame, length))
    {
        stats.invalid_frames++;
        return;
    }

    if ((read_u16(frame + kPacketNumberOffset) != expected_packet_number) ||
        (read_u16(frame + kTargetAddressOffset) != expected_response_target) ||
        (read_u16(frame + kSourceAddressOffset) != expected_response_source))
    {
        return;
    }

    const uint8_t *payload = frame + kLongPayloadOffset;
    const uint16_t offset = read_u16(payload + 8u);
    const uint16_t chunk_length = read_u16(payload + 10u);

    stats.response_frames++;

    if ((chunk_length == 0u) || (chunk_length > kSignatureChunkLength) ||
        ((offset % kSignatureChunkLength) != 0u) ||
        (offset >= (kSignatureChunkLength * kSignatureChunkCount)))
    {
        return;
    }

    response_mask |= (uint8_t)(1u << (offset / kSignatureChunkLength));
    if (response_mask == 0x0Fu)
    {
        stats.completed++;
        release_transaction();
    }
}

} // namespace

void auth_proxy_init(void)
{
    rx_head = 0u;
    rx_tail = 0u;
    memset(&stats, 0, sizeof(stats));
    release_transaction();

    bus_port_to_host.set_frame_capture(auth_proxy_filter_frame,
                                       auth_proxy_capture_frame);
}

uint8_t auth_proxy_filter_frame(const uint8_t *data, uint16_t length)
{
    return is_auth_command_frame(data, length) ? 1u : 0u;
}

void auth_proxy_capture_frame(const uint8_t *data, uint16_t length)
{
    if ((data == nullptr) || (length == 0u) || (length > BMCU_AUTH_PROXY_FRAME_MAX))
    {
        return;
    }

    const uint8_t next = queue_next(rx_head);
    if (next == rx_tail)
    {
        stats.queue_drops++;
        return;
    }

    QueuedFrame &slot = rx_queue[rx_head];
    slot.length = length;
    memcpy(slot.data, data, length);
    __asm__ volatile("" ::: "memory");
    rx_head = next;
}

void auth_proxy_run(void)
{
    const uint32_t now = time_ticks32();

    while (rx_tail != rx_head)
    {
        const uint8_t tail = rx_tail;
        const uint16_t length = rx_queue[tail].length;
        const uint8_t *frame = rx_queue[tail].data;

        if (is_challenge_frame(frame, length))
        {
            begin_transaction(frame, length);
        }
        else if (((state == ProxyState::send_forward) ||
                  (state == ProxyState::wait_response)) &&
                 is_response_frame(frame, length))
        {
            note_response(frame, length);
        }

        rx_tail = queue_next(tail);
        if (state == ProxyState::idle)
        {
            break;
        }
    }

    if ((state == ProxyState::send_forward) && bus_port_to_host.idle)
    {
        bus_port_to_host.send_package(forward_frame, forward_length);
        stats.forwards_sent++;
        state = ProxyState::wait_response;
        response_deadline = now + ms_to_ticks32(BMCU_AUTH_PROXY_TIMEOUT_MS);
    }

    if ((state == ProxyState::send_forward) &&
        time_reached32(now, send_deadline))
    {
        if (retries_left > 0u)
        {
            retries_left--;
        }
        else
        {
            stats.timeouts++;
            release_transaction();
        }
    }
    else if ((state == ProxyState::wait_response) &&
             time_reached32(now, response_deadline))
    {
        if (retries_left > 0u)
        {
            retries_left--;
            state = ProxyState::send_forward;
            send_deadline = now + ms_to_ticks32(BMCU_AUTH_PROXY_TIMEOUT_MS);
        }
        else
        {
            stats.timeouts++;
            release_transaction();
        }
    }
}

uint8_t auth_proxy_tx_locked(void)
{
    return tx_locked;
}

void auth_proxy_get_stats(AuthProxyStats *out)
{
    if (out == nullptr)
    {
        return;
    }

    const uint32_t irq = irq_save_wch();
    *out = stats;
    irq_restore_wch(irq);
}

#else

void auth_proxy_init(void) {}
void auth_proxy_run(void) {}
uint8_t auth_proxy_tx_locked(void) { return 0u; }
uint8_t auth_proxy_filter_frame(const uint8_t *, uint16_t) { return 0u; }
void auth_proxy_capture_frame(const uint8_t *, uint16_t) {}
void auth_proxy_get_stats(AuthProxyStats *stats)
{
    if (stats != nullptr)
    {
        memset(stats, 0, sizeof(*stats));
    }
}

#endif
