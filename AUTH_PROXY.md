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
Retry count after timeout:  1
BMCU AMS identity:          AMS08, 10.50.00.00
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

That environment is identical to `a1_auth_proxy`, except the `0x0103`
identity response becomes:

```text
device name: N3F05
version:     90.00.00.00
wire bytes:  00 00 00 5A
```

The two environments intentionally remain separate so the auth result can be
compared without conflating it with the model/version compatibility branch.

The existing release matrix is unchanged. A separate GitHub Actions job is
provided for the two experimental builds.

## GitHub Actions

The workflow in `.github/workflows/build.yml` includes a separate
`build_auth_proxy` job. After pushing a branch or opening a pull request it
builds both environments and publishes:

```text
auth-proxy-firmwares-<commit>/
  a1_auth_proxy_AMS08_10.50.00.00.bin
  a1_auth_proxy_AMS2Pro_90.00.00.00.bin
```

Open the workflow run and download the `auth-proxy-firmwares-<commit>`
artifact from the Actions page.

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
  `AMS08 + 10.50.00.00` identity may trigger a compatibility warning even when
  the random challenge succeeds; test `a1_auth_proxy_ams2` to isolate it.
- `0x040D` certificate behavior is left untouched. If a tested firmware
  requires certificate relay as well, it must be captured and added as a
  separate transaction stage.
- The default selector rewrite is based on the observed challenge layout.
  If the official AMS ignores the forwarded request, test with rewriting
  disabled before changing byte offsets.
- The single RS485 port cannot hide the official AMS's direct response. This
  design assumes `1.05-1.07` accepts a valid response without binding it to
  the challenged device.
- If the printer later rejects the official AMS source identity, a two-port
  isolated bridge is required.
- Do not hot-plug AMS/BMCU hardware while the printer is powered.
- This is a research build. Verify startup, repeated reboot, second print,
  filament loading, unloading, and multi-AMS enumeration before using it for
  actual printing.
