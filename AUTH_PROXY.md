# A1 Shared-Bus Authentication Relay

This branch copy adds an opt-in authentication relay for the A1 printer
firmware range where the `0x040E` challenge response is not bound to the AMS
that was challenged.

The normal firmware environments remain unchanged. Use the dedicated
`a1_auth_proxy` PlatformIO environment for testing.

## Data flow

```text
printer -- 0x040E challenge --> BMCU
BMCU copies the challenge and optionally rewrites its AMS selector
BMCU retransmits the valid challenge frame on the shared RS485 bus
official AMS signs the challenge and returns four 64-byte chunks
printer accepts the valid response under the affected firmware policy
BMCU resumes its normal heartbeat/filament/motion traffic
```

The random challenge bytes are never changed. Only routing metadata,
transaction identity, and CRCs may be rewritten.

## Default build settings

`platformio.ini` defines `env:a1_auth_proxy` with:

```text
BMCU AMS index:             1
Official AMS index:         0
Rewrite challenge selector: enabled
Challenge selector byte:    payload offset 4
Response timeout:           30 ms
Direct-response grace:      5 ms
Retry count after timeout:  1
BMCU AMS identity:          AMS08, 01.00.06.87
Bus heartbeat tolerance:    1000 ms
```

Change these values to match the actual test setup:

```text
BAMBU_BUS_AMS_NUM
BMCU_AUTH_PROXY_TARGET_AMS_NUM
BMCU_AUTH_PROXY_FILTER_LOCAL_INDEX
BMCU_AUTH_PROXY_REWRITE_DEVICE_INDEX
BMCU_AUTH_PROXY_DEVICE_INDEX_OFFSET
BMCU_AUTH_PROXY_BUMP_PACKET_NUMBER
```

The BMCU and signing AMS indexes must be different.

## Build

```bash
pio run -e a1_auth_proxy
```

The generated image is:

```text
.pio/build/a1_auth_proxy/firmware.bin
```

For the AMS 2 Pro identity, use:

```bash
pio run -e a1_auth_proxy_ams2
```

`a1_auth_proxy_ams2` uses the recognized 57-byte N3F05 reply. It is
identical to `a1_auth_proxy` except the `0x0103` identity becomes:

```text
device name: N3F05
version:     90.00.00.00
wire bytes:  00 00 00 5A
reply size:  57 bytes
```

### Authentication A/B verification

GitHub Actions publishes two firmware images:

```text
PROXY_AMS0_N3F05_90.00.00.00_HT_FULL57.bin
    N3F05 + 90.00.00.00
    AMS index 0
    authentication proxy enabled, signing target address 0x1800
    57-byte AMS2 identity response

PROXY_AMS0_N3F05_90.00.00.00_HT_ORIGINAL21.bin
    N3F05 + 90.00.00.00
    AMS index 0
    authentication proxy enabled, signing target address 0x1800
    original 21-byte AMS2PRO identity array
```

Use the same AMS index, official AMS position, filament state, and power cycle
for both tests.

Interpretation:

```text
A clears the warning, but load/print or second print later fails
    -> model/version spoof only hides the prompt; proxy is doing real work

A and B behave identically across reboot and repeated prints
    -> the relay is not required on that firmware, or official AMS auth is
       already accepted globally

A clears the warning and B restores immediate recognition/runtime
    -> proxy works but needs further timing fixes

Both still show the mismatch warning
    -> the warning is not being cleared by the AMS2 identity alone
```

For definitive proof, capture Bambu Bus during both boot sequences and record
whether the BMCU device index receives its own `0x040D/0x040E` transaction and
which device answers it.

The existing release matrix is unchanged. A separate GitHub Actions job is
provided for the AMS2 Pro authentication build.

## GitHub Actions

The workflow in `.github/workflows/build.yml` includes a separate
`build_auth_proxy` job. After pushing a branch or opening a pull request it
builds and publishes:

```text
auth-proxy-firmwares-<commit>/
  PROXY_AMS0_N3F05_90.00.00.00_HT_FULL57.bin
  PROXY_AMS0_N3F05_90.00.00.00_HT_ORIGINAL21.bin
```

Open the workflow run and download the `auth-proxy-firmwares-<commit>`
artifact from the Actions page.

The full release matrix is intentionally manual. Open
`Actions > 自动编译固件 > Run workflow` to run the `手动编译全部固件` job.
This prevents the long release job from delaying or marking authentication
relay test runs as failed.

## Expected counters

`auth_proxy_get_stats()` exposes:

```text
challenges_seen
forwards_sent
response_frames
completed
timeouts
wrong_index
queue_drops
invalid_frames
```

A successful transaction should normally show:

```text
challenges_seen = 1 or more
forwards_sent = challenges_seen
response_frames = 4 per completed transaction
completed = 1 or more
queue_drops = 0
invalid_frames = 0
```

## Important limitations

- This implementation targets the observed `0x040E` challenge transaction.
  It does not forge or copy a private key.
- Version/model identity and authentication are separate layers. The
  `AMS08 + 01.00.06.87` identity may trigger a compatibility warning even
  when the random challenge succeeds; test `a1_auth_proxy_ams2` to isolate it.
- `a1_auth_proxy_ams2` is the only automatic Actions build. It uses the
  recognized 57-byte reply, 5 ms response grace, and 1000 ms heartbeat limit.
- `0x040D` certificate behavior is left untouched. If a tested firmware
  requires certificate relay as well, it must be captured and added as a
  separate transaction stage.
- The default selector rewrite is based on the observed challenge layout.
  If the official AMS ignores the forwarded request, test with rewriting
  disabled before changing byte offsets.
- The single RS485 port cannot hide the official AMS's direct response. This
  design assumes `1.05-1.07` accepts a valid response without binding it to
  the challenged device.
- AMS HT uses the `0x1800` bus address. Use the `_HT` authentication-proxy
  build when AMS HT is the only official signing device. The regular proxy
  build targets normal AMS address `0x0700`.
- The released proxy image uses AMS index 0 because the stock BMCU is assigned
  AMS index 1. One BMCU proxy does not automatically authenticate another BMCU
  as a separate AMS entry.
- A 5 ms grace period lets the official AMS answer the original challenge
  before BMCU retransmits it. This reduces duplicate signatures and brief
  heartbeat contention on the shared bus.
- Do not increase heartbeat tolerance to 2500 ms on this firmware path:
  hardware testing showed that the BMCU then stops enumerating. Experimental
  builds therefore retain the original 1000 ms value.
- If the printer later rejects the official AMS source identity, a two-port
  isolated bridge is required.
- Do not hot-plug AMS/BMCU hardware while the printer is powered.
- This is a research build. Verify startup, repeated reboot, second print,
  filament loading, unloading, and multi-AMS enumeration before using it for
  actual printing.
