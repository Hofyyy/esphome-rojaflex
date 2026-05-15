# Rojaflex Shutter Control via ESPHome

ESPHome firmware for controlling Rojaflex 433.92 MHz shutters (P109 protocol) through a CC1101 transceiver on a LilyGO T-Embed CC1101 (ESP32-S3). Exposes each shutter as a `cover` entity in Home Assistant with a 0..100 % position slider.

## Features

- Single shared housecode for all shutters (7 hex chars)
- Channels `2..9` by default; the protocol supports `0..15` (channel `0` is broadcast)
- Commands: `up`, `down`, `stop`, plus emulated `set_position` (mid-position)
- Packet format: P109 / 9-byte payload (18 hex chars), no payload CRC (HW CRC ein)
- TX repetitions set in YAML (`tx_repetitions:`, default `2`, range `1..4`, 80 ms gap between repeats)
- Auto-learn of the housecode while the configured housecode is `0000000` (3 matching frames → adopted, persisted to NVS)
- Position tracking, in this priority:
  1. **Motor feedback (DEV=5)** — actual position reported by the motor
  2. **Foreign-remote sniff (DEV=A)** — a second physical remote takes priority
  3. **Optimistic TX update** — we write the target end stop on successful UP/DOWN
- 200 ms self-TX echo guard so our own broadcasts don't poison the foreign-remote path
- Per-channel **auto-calibration** of `time_to_open_s` / `time_to_close_s` (min 5 s sanity floor; motor feedback must arrive within 60 s of the TX), persisted to NVS reactively
- Mid-position fahrten emulated FHEM-style: UP/DOWN + scheduled STOP after calculated duration
- Live linear interpolation animates the HA slider during timed runs (500 ms tick)
- Channel 0 hard-locked to end-stops only (no mid-position, no calibration capture)
- Gesture-based per-channel calibration reset: 5× STOP on the HA cover within 2 s wipes that channel's cal (physical-remote STOPs are ignored)

## Hardware

T-Embed CC1101 / LilyGO factory pinout (matches `examples/cc1101_recv_irq` and `utilities.h` in the [Xinyuan-LilyGO/T-Embed-CC1101](https://github.com/Xinyuan-LilyGO/T-Embed-CC1101) repo):

| Signal | GPIO | Note |
|---|---|---|
| SPI CLK | 11 | shared with display / SD |
| SPI MOSI | 9 | |
| SPI MISO | 10 | |
| CC1101 CS | 12 | `BOARD_LORA_CS` |
| CC1101 GDO0 (IRQ) | 3 | `BOARD_LORA_IO0` |
| Board Power Enable | 15 | `BOARD_PWR_EN` — driven HIGH on boot |
| RF switch 433 MHz | 47, 48 | `BOARD_LORA_SW1` / `SW0` — both HIGH for ~434 MHz |

## Files

- `esphome-rojaflex-native.yaml` — top-level ESPHome config (the only YAML you compile)
- `components/rojaflex/` — external_component, hub + cover + sub-platforms
  - `rojaflex.{h,cpp}` — hub class (CC1101Listener, RX routing, AutoLearn, calibration, NVS, gesture reset)
  - `p109_protocol.h` — pure protocol layer (P109 frame build/decode, housecode, motion plan)
  - `cover/` — per-channel `RojaflexCover`
  - `text_sensor/` — sub-platform wiring text-sensor entities into the hub
- `secrets.yaml` — WiFi + OTA credentials (gitignored; copy `secrets.yaml.example`)
- `tests/components/rojaflex/` — component test YAMLs in the ESPHome upstream-PR layout (Arduino + ESP-IDF on ESP32 and ESP32-S3)

## Build notes

- Framework is **esp-idf** (`framework: type: esp-idf`, `version: recommended`). The component itself uses only stock ESPHome APIs and the upstream `cc1101` component — no Arduino-specific calls. The switch from arduino to esp-idf was verified on the T-Embed CC1101 on 2026-05-15: firmware.bin shrank by ~88 kB / −10.6 % (and static RAM by ~1.5 kB), OTA upload ~0.86 s faster due to the smaller image. Boot time measured identical between the two frameworks on this hardware — the perceived speedup is the OTA, not the boot. Arduino still builds (the upstream cc1101 + our component are framework-portable) but is no longer the default.

## Use as external component

To pull this component into your own ESPHome config without cloning the repo, add:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/Hofyyy/esphome-rojaflex
    components: [rojaflex]
```

Then declare the `cc1101:` and `rojaflex:` hubs, plus per-channel `cover:` / `text_sensor:` entries — see [`esphome-rojaflex-native.yaml`](esphome-rojaflex-native.yaml) for a complete example.

## Setup

### 1. Secrets

Copy `secrets.yaml.example` to `secrets.yaml` next to the main YAML and fill in your values:

```yaml
wifi_ssid: "Your_WiFi"
wifi_password: "Your_Password"
ota_password: "your_ota_password"
```

### 2. Flash

USB (first time):

```bash
esphome run esphome-rojaflex-native.yaml --device /dev/cu.usbmodem1101 --no-logs
```

OTA (after first flash) — always compile and upload separately, `esphome upload` alone does NOT recompile:

```bash
esphome compile esphome-rojaflex-native.yaml && \
esphome upload  esphome-rojaflex-native.yaml --device rojaflex-shutter.local
```

`--no-logs` prevents the post-flash log tail from blocking the terminal. Drop it if you want to see boot output.

### 3. Set the housecode

Either call the `set_housecode` service from HA:

```json
{ "new_housecode": "C742CD9" }
```

Or leave the default `0000000` and press a button on your existing Rojaflex remote a few times. After 3 matching frames the housecode is auto-learned. The `Housecode` text sensor walks through `Receiving...` → `Learning: 0x… (n/3)` → `0xC742CD9`.

### 4. Calibrate each channel

Mid-position fahrten need both travel directions calibrated per channel. Bootstrap needs one full open + one full close per shutter:

1. Watch `Rolladen <N> Status`. It starts as `Calibration needed: drive fully open or closed once`.
2. Press `close` once. Status moves to `Calibration needed: drive fully open once` after the motor confirms the bottom end stop via DEV=5.
3. Press `open` once. The motor reports the top end stop; `time_to_open_s` is captured.
4. Repeat the other direction. Status becomes `Calibrated` and from then on shows the last frame on the channel as `<Cmd> (<rssi> dBm)`.
5. The slider can now be dragged to any percentage.

Calibration values are persisted to NVS the moment they're learned and survive reboots. To wipe calibration for a single channel and start fresh, tap that cover's **STOP button in HA 5 times within 2 seconds**. The hub clears that channel's cal and the status sensor flips back to `Calibration needed: ...`. The gesture only listens to HA-dashboard STOPs — panic-mashing the physical remote never triggers a reset.

## Home Assistant entities

Per channel `<N>`:

- `cover.rolladen_<N>` — cover with `has_position`, slider 0..100 %
- `text_sensor.rolladen_<N>_status` — calibration state, then live RX activity once calibrated

Diagnose (collapsed by default in HA):

- `text_sensor.housecode` — combined status display: `Receiving...` → `Learning: 0x<hc> (n/3)` → `0x<hc>` once locked in
- `text_sensor.cc1101_last_tx` (disabled by default) — last TX outcome: `ok` if any retry was acknowledged by the CC1101 driver, otherwise the short CC1101Error name — `timeout` (chip didn't return to IDLE after TX strobe), `params` (config bug), or `pll-lock` (synthesizer didn't lock)
- `text_sensor.rojaflex_last_rx_info` (disabled by default) — decoded frame info; re-enable when inspecting RX frames for debugging
- `text_sensor.wifi_ssid` — currently associated AP
- `sensor.wifi_signal` — WiFi signal strength in dBm, polled every 60 s

There is no diagnostic reset button — calibration for a single channel is wiped by tapping its HA cover STOP button 5 times within 2 seconds (see Setup step 4).

## Services

| Service | Payload | Effect |
|---|---|---|
| `set_housecode` | `{ "new_housecode": "C742CD9" }` | Manually override the shared housecode |
| `get_housecode` | — | Logs the current housecode |
| `test_send` | `{ "channel_id": 2, "command": 8 }` | Low-level frame test. `command`: `0` stop, `1` up, `8` down |

## Per-channel status sensor

| State | Text |
|---|---|
| Neither direction calibrated | `Calibration needed: drive both end stops once` |
| Open direction missing | `Calibration needed: drive fully open once` |
| Close direction missing | `Calibration needed: drive fully closed once` |
| Calibrated, position not synced since boot | `Resync: drive once` |
| Calibrated, no RX yet | `Calibrated` |
| Calibrated, RX seen | e.g. `Stop (-64 dBm)`, `Up (-50 dBm)`, `Pos. 45% (-72 dBm)` |
| Channel 0 (broadcast slot) | `Broadcast channel - end stops only` |

Once calibrated, the sensor shows the last frame received on that channel (remote presses, motor feedback, and channel-0 broadcasts — which appear on every shutter). Updates are reactive on RX; a 5 s safety poll handles the rare non-RX state transitions (gesture reset). If a particular motor never sends DEV=5, calibration cannot complete and the slider will only ever offer 0 % and 100 % — end-stop fahrten still work.

## Mid-position behavior

- A new mid-position request supersedes any in-flight one (per-channel scheduler slot).
- A foreign-remote frame for the channel cancels the scheduled STOP and any calibration capture — the physical user takes priority.
- Channel 0 mid-position requests are rejected with a log warning.

## Protocol reference (P109)

18-hex-char string, 9 bytes on air:

```
P109# 08 C742CD9 9 8 A 01 8 A 8E
      └─ └───┬───┘ └ └ └ └─ └ └─┬─┘
   device  housecode ch cmd dev ... checksum
```

| Pos | Len | Field | Example |
|---|---|---|---|
| 0..4 | 5 | Protocol prefix | `P109#` |
| 5..6 | 2 | Device ID | `08` |
| 7..13 | 7 | Housecode (hex) | `C742CD9` |
| 14 | 1 | Channel (`0..F`) | `9` |
| 15 | 1 | Command | `8` (DOWN) |
| 16 | 1 | Device type | `A` (remote) |
| 17..18 | 2 | Reserved | `01` |
| 19 | 1 | Command repeat | `8` |
| 20 | 1 | Device byte | `A` |
| 21..22 | 2 | Checksum | `8E` |

**Command codes:** `0` stop, `1` up, `8` down, `9` save favorite, `D` go-to favorite, `E` status request.

**Checksum:** 8-bit sum of bytes at hex positions 7..19 (each pair = one byte), masked to 8 bits.

**Radio settings:** 433.92 MHz, GFSK, ~9.99 kbps, sync word `D391`.

## Troubleshooting

- **No entities in HA:** check the device is online and on WiFi; restart the ESPHome integration.
- **Shutter doesn't react:** verify the housecode (`Housecode` text sensor shows `0x<hc>`), check CC1101 wiring, raise `tx_repetitions` in the YAML, check antenna/distance. Watch `CC1101 Last TX` — if it stays on `timeout` or `pll-lock` you have a hardware issue, not a range issue.
- **Slider snaps to 0/100 only:** the channel isn't calibrated yet — see step 4 of Setup. If the motor never sends DEV=5, mid-position isn't possible for that motor.
- **Auto-learn doesn't fire:** confirm `Housecode` still shows `Receiving...`, press the same button multiple times within close range, watch for `Learning: 0x<hc> (n/3)`.
- **Weak RSSI on status sensor (e.g. `Up (-95 dBm)`):** move the controller closer to the remote/motor or improve the antenna.

Verbose protocol logs:

```yaml
logger:
  level: DEBUG
  logs:
    rojaflex: DEBUG
    cc1101: VERBOSE
```

## References

- FHEM reference implementation: [RFFHEM `10_SD_Rojaflex.pm`](https://github.com/RFD-FHEM/RFFHEM/blob/master/FHEM/10_SD_Rojaflex.pm)
- Protocol data (RFFHEM): [`SD_Protocols/Data.pm`](https://github.com/RFD-FHEM/RFFHEM/blob/master/lib/FHEM/Devices/SIGNALduino/SD_Protocols/Data.pm)
- rtl_433 decoder: [`src/devices/rojaflex.c`](https://github.com/merbanan/rtl_433/blob/master/src/devices/rojaflex.c)
- T-Embed CC1101 board: [Xinyuan-LilyGO/T-Embed-CC1101](https://github.com/Xinyuan-LilyGO/T-Embed-CC1101)
- Upstream ESPHome cc1101 component: <https://github.com/esphome/esphome/tree/dev/esphome/components/cc1101>
- ESPHome docs: <https://esphome.io/>
