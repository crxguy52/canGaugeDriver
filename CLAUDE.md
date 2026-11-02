# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Arduino firmware that reads HS-GMLAN (500 kbps) from a GM ECU (LFX V6, E92 ECM) and drives the stock NB Miata cluster: tach, speedo, coolant gauge, and warning lights. It will also command the ECM's AC compressor by impersonating the BCM/Platform (not implemented yet in firmware; prototyped in Python). Used with a separate [baseboard](https://github.com/crxguy52/canGagueDriver-baseBoard) that level-shifts 12 V ↔ 5 V.

**Target hardware:** the Seeed/Longan **CANBed**, which is an ATmega32U4 with an MCP2515 for classic CAN. A port to the **CANBed FD** is planned. The FD board uses the same 32U4 but an MCP2517FD CAN chip, so the port means swapping the vendored `Longan_CAN_MCP2515` library and redoing the RX mask/filter setup. The timer, ISR, and pin code should carry over unchanged.

The code is AVR-specific: it writes Timer0/Timer1 registers and `OCR1C` directly.

## Build / flash

There is no build script. Use the Arduino IDE or arduino-cli with a Leonardo-compatible FQBN:

```
arduino-cli compile --fqbn arduino:avr:leonardo .
arduino-cli upload  --fqbn arduino:avr:leonardo -p <PORT> .
```

The sketch is `canGaugeDriver.ino`. Everything under `src/` is compiled recursively, including the vendored `src/Longan_CAN_MCP2515` library. Don't install a separate MCP2515 library: it would conflict with the vendored one.

There are no unit tests for the firmware.

## Python bench tools (`test_scripts/`)

Managed with PDM (`pdm install`, then `pdm run python <script>`). Both scripts use the `socketcan` interface on `can0`, so they need Linux with a SocketCAN adapter.

- `replay_logfile.py`: replays the recorded `GMLAN_*.blf` onto the bus so the firmware can be bench-tested without the car. Run it from the repo root, because the log path is relative.
- `ac_compressor_cycle.py`: prototype of the Platform→ECM AC request (frames 0x1F1 at 100 ms and 0x4E9 at 1 s, ECM replies in 0x3D1). It supports `--debug`, which runs without hardware. It imports `cantools`, which is **not** in `pyproject.toml`, and loads a DBC from `../../../dbc/gmlan_v1.7_custom.dbc`, which is outside this repo.

## Firmware architecture

Control flow:

```
Timer0 COMPA ISR (MAIN_INT_FREQ_HZ) ──sets──> flag_mainloop
loop(): on flag_mainloop
  ├─ CAN.checkReceive → handle_received_message()  (src/can.cpp)
  │     decode into global EngineData; set lights, coolant PWM directly
  ├─ every COUNT_INTVL_V_BUS ticks: read ADCs (vbus, fuel), CEL in, AC req;
  │     TX vbus on ID_V_BUS (0x780); alternator light
  ├─ two more scheduled slots (Platform Gen Status / Cfg Data): empty stubs for AC
  └─ warn_blink_flag = any(flag_oil_low, flag_cool_hot, flag_v_low)

Timer1 (free-running 16-bit, normal mode), three compare channels:
  COMPA → toggle tach pin,  OCR1A += count_intvl_tach
  COMPB → toggle speedo pin, OCR1B += count_intvl_speed
  COMPC → toggle SRS pin as warning blinker
```

Key points that span files:

- **Timer0 is taken over** (`timer0_init`), so Arduino `millis()`, `delay()`, and `micros()` are wrong or broken after `setup()` calls it. Schedule work with `loopCount` and the `COUNT_INTVL_*` constants instead.
- **Gauge frequency output** works by adding a half-period to the compare register on each ISR (`OCR1x += interval`). The interval math (`TIMER1_FREQ/(2*rpm*RPM2HZ)`) is currently commented out in `src/can.cpp`, so the tach and speedo run at the fixed `TIMER1_COMPARE_VALUE` interval.
- **ISRs use `digitalWrite`/`digitalRead`** on the `hwdefs.h` pins, so pins are defined in one place. This costs a few µs per toggle, which is acceptable at gauge frequencies.
- **Globals shared with ISRs** are defined in the `.ino` and `extern`-declared in `src/interrupts.h`. `engData` is read inside ISRs but is not `volatile`.
- **CAN RX filtering** is done in hardware by the MCP2515 masks and filters in `setup()`. RXB0 takes RPM (`ID_ENGINE_GENERAL_STATUS_1`, the fastest message), and RXB1 takes the other three. To add an RX message, update the masks and filters there *and* add a `case` in `handle_received_message()`.
- **The timing constants form a chain.** In `src/timers.h`, the preprocessor picks the Timer0 prescaler from `MAIN_INT_FREQ_HZ` and `F_CPU`, and every `COUNT_INTVL_*` value is derived from `MAIN_INT_FREQ_HZ`. Change frequencies there, not in the derived values.

Where configuration lives:

| File | Contents |
|---|---|
| `src/hwdefs.h` / `hwdefs.cpp` | pins, CAN IDs, scaling factors, fuel-sender lookup table (ADC counts → gallons, used with `interp1f`) |
| `src/limits.h` | coolant gauge PWM breakpoints, oil-pressure / bus-voltage / shift-light thresholds, min RPM/MPH for output |
| `src/timers.h` | loop and timer frequencies, prescalers |
| `src/typedefs.h` | `EngineData`, the single shared state struct |

The CAN signal decoding (byte offsets and scaling) in `src/can.cpp` follows GM GMLAN definitions (GMW8762 / GMW8771). Verify any new signal against the DBC or the spec before you add it.
