# ESP32-S3 Voice Assistant — Firmware Design

**Date:** 2026-03-27
**Hardware:** WaveShare ESP32-S3-Audio-Board
**Scope:** Firmware only (companion server is a separate project)

---

## Project Structure

Project lives at `~/projects/voice-assistant/`, organised alongside the future companion server.

```
~/projects/voice-assistant/
├── firmware/                   # ESP-IDF project (this spec)
│   ├── main/
│   │   └── main.c              # App entry, init, state machine
│   ├── components/
│   │   ├── audio_pipeline/     # Mic capture, codec, resampling
│   │   ├── wake_word/          # ESP-SR integration
│   │   ├── led_indicator/      # RGB LED state feedback
│   │   └── ws_transport/       # WiFi + WebSocket streaming
│   ├── CMakeLists.txt
│   └── sdkconfig
├── server/                     # Companion server (future)
└── docs/
    └── specs/                  # This file lives here
```

---

## Audio Pipeline

Built using ESP-ADF's element chain:

```
I2S mic input → ADF resample → ring buffer → wake word detect
                                           ↘ (after wake) WebSocket stream out
```

- **Codec:** ES8388 (onboard WaveShare codec, configured via I2C)
- **Sample rate:** 16kHz mono — minimum for ESP-SR, matches Whisper input requirements, minimises bandwidth
- **Ring buffer:** ~500ms capacity, decouples capture from processing

---

## State Machine

Five states driven by ESP-SR events and WebSocket callbacks:

```
IDLE ──(wake word)──→ LISTENING ──(silence/timeout)──→ STREAMING
                                                            │
                    SPEAKING ←──(audio received)────── WAITING
                        │
                    (playback done)
                        ↓
                      IDLE
```

| State | Description |
|-------|-------------|
| IDLE | ESP-SR running, awaiting wake word |
| LISTENING | Recording to ring buffer |
| STREAMING | Sending audio chunks over WebSocket |
| WAITING | WebSocket open, awaiting server TTS response |
| SPEAKING | Playing TTS audio back via I2S |

**Timeouts** (configurable via `sdkconfig`):
- Max recording duration: 8 seconds
- Max server response wait: 30 seconds

Both timeouts return the device to IDLE.

---

## WebSocket Transport

Local network only — no authentication required.

- **Protocol:** WebSocket over TCP
- **Audio frames:** Binary, raw PCM 16kHz mono 16-bit
- **Control frames:** JSON text

### Control Messages

| Direction | Message | Meaning |
|-----------|---------|---------|
| Device → Server | `{"type": "start"}` | Start of audio stream |
| Device → Server | `{"type": "end"}` | End of recording |
| Server → Device | `{"type": "tts_done"}` | TTS audio fully sent |

### Connection Management

- Connect on boot
- Reconnect with exponential backoff on disconnect
- WiFi credentials stored in `sdkconfig` (menuconfig), not hardcoded

---

## LED Indicator

Onboard RGB LED via GPIO, managed by the `led_indicator` component.

| State | Color | Pattern |
|-------|-------|---------|
| IDLE | Off | — |
| LISTENING | Blue | Solid |
| STREAMING | Blue | Slow pulse |
| WAITING | Yellow | Slow pulse |
| SPEAKING | Green | Solid |
| Error / reconnecting | Red | Fast blink |

API: `led_set_state(state)` — called directly by the state machine, no separate task.

---

## Build Environment

- **ESP-IDF:** v5.5.2 (`~/.espressif/` → `/mnt/overflow/.espressif/`)
- **ESP-ADF:** `~/esp/esp-adf/` → `/mnt/overflow/esp/esp-adf/`
- **Wake word engine:** ESP-SR (bundled with ESP-ADF)
- **TTS on companion server:** Piper (local, no API cost)

---

## Out of Scope (Firmware)

- Companion server implementation (separate project)
- OTA firmware updates
- Multi-room / multi-device
- Custom wake word training
