# Sprint timing gate firmware

XIAO ESP32-C3 on every post. Emitter posts send 38 kHz IR bursts. Receiver posts timestamp the beam break and report to the phone over BLE.

## Status

| Piece | State |
|---|---|
| `include/gate_core.h` (detector, packets, sync math) | Written and tested on a laptop |
| `host_tests/` | 6 test groups pass: `make -C host_tests` |
| `src/emitter.cpp`, `src/receiver.cpp` | **Written, never compiled, never run on hardware** |
| `app/index.html` (phone page) | Written. Protocol code checked against the C++ packets and run logic smoke-tested with a stubbed DOM. **Never run on a real phone.** |

## Phone page

`app/index.html` is one file: connect to the start and finish receivers, sync clocks, arm, and see the split. It also has a "Phone start" mode (beep, then time to the finish gate) for when you only have one receiver.

It uses Web Bluetooth, so it works in Chrome on Android (and Chrome/Edge on a computer), not on iPhone. Browsers only allow Bluetooth from a secure page, so host it over https. The easiest way is GitHub Pages: put `app/index.html` in a repo, turn on Pages, and open the link on your phone. Opening the file straight from storage may not work.

Each Arm tap re-syncs the clocks, then arms. The receiver disarms itself after one trigger, so tap Arm before every run.

Protocol check: `cd host_tests && g++ -std=c++11 -I../include -o gen_packets gen_packets.cpp && ./gen_packets | node test_page_core.js`

Expect a few compile errors the first time, mostly NimBLE API differences between versions.

## Layout

    include/gate_core.h    all logic that does not touch hardware
    src/emitter.cpp        LEDC 38 kHz carrier + burst timer
    src/receiver.cpp       pulse ISR, trigger polling, BLE, serial bench commands
    host_tests/            simulations, no hardware needed
    platformio.ini         one env per post type

## Pins (XIAO ESP32-C3)

| Post | Pin | Goes to |
|---|---|---|
| Emitter | D2 (GPIO4) | 1 kOhm, then 2N3904 base |
| Receiver | D2 (GPIO4) | TSOP38238 OUT |

Check the 2N3904 pinout and the TSOP38238 supply filter against the datasheets before wiring.

## First day with parts

1. `pio run -e emitter -t upload` on one board, `pio run -e receiver -t upload` on the other.
2. Open the receiver's serial monitor. Status prints twice a second:
   `beam=present armed=0 pulses/s=830 (expect 833)`
3. Wave a hand through the beam. Type `a` first to arm; you should see `TRIGGER #1 at ... us`.
4. If `pulses/s` is far below expected or the state flips between searching and present, fix that before anything else: alignment, the supply filter, or the burst timing in `Config`.

## Constants to check against the datasheet

All in `Config` in `gate_core.h`:

- `burst_period_us` (1200): 600 us on, 600 us off. The TSOP38238 needs a minimum burst length and gap; confirm.
- `break_gap_us` (6000): 5 missed pulses means "broken". Lower is faster but triggers on sun dropouts. Tune outdoors.
- `trigger_offset_us` (600): the true break is somewhere in the one period after the last pulse, so this centers the error. Calibrate against a reference.

## BLE protocol

Service `7a1c0001-6b1e-4c1d-9d3a-5f6e7d8c9b01`, command characteristic `...0002` (write), event characteristic `...0003` (notify). Packet layouts are documented in `gate_core.h` under `proto`. Until the phone app exists you can poke at it with nRF Connect or LightBlue.

## Known limits

- Sync over BLE is only good to a few milliseconds (see the test output). If two receivers on one beam disagree by more than that, the next step is ESP-NOW between posts, which is a separate piece of firmware.
- Pulse timestamps are taken in a GPIO interrupt, so BLE traffic should not move them, but this is untested.
- Burst gating uses `esp_timer` in task context; if a scope shows irregular bursts, move it to RMT.
