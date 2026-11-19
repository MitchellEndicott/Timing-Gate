// gate_core.h
// Hardware-independent logic for the sprint timing gates.
// Plain C++11, no Arduino or ESP-IDF includes, so it compiles on a laptop for
// testing and on the XIAO ESP32-C3 unchanged.
//
// Contents:
//   1. Config          - every timing constant in one place
//   2. BeamDetector    - missing-pulse trigger logic (runs on the receiver)
//   3. proto           - BLE packet layout (pack/unpack, little-endian)
//   4. estimate_offset - clock-offset math (reference for the phone app)
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace gate {

// ---------------------------------------------------------------------------
// 1. Config
// ---------------------------------------------------------------------------
// VERIFY against the TSOP38238 datasheet before trusting these. The module
// expects a burst of at least ~10 carrier cycles followed by a gap, and it
// filters out continuous 38 kHz. 600 us on / 600 us off is about 23 cycles per
// burst, which is the usual starting point.
struct Config {
    uint32_t burst_period_us;   // emitter: one burst + one gap
    uint32_t break_gap_us;      // receiver: no pulse for this long => beam broken
    uint32_t acquire_pulses;    // receiver: pulses in a row before "beam present"
    uint32_t lockout_us;        // receiver: ignore re-triggers this long after a trigger
    uint32_t trigger_offset_us; // receiver: added to the last pulse time

    Config()
        : burst_period_us(1200),
          break_gap_us(6000),       // 5 missed pulses. Larger = fewer false triggers, more latency
          acquire_pulses(10),
          lockout_us(1000000),      // the plan's "about a second"
          trigger_offset_us(600) {} // half a period: the break happened somewhere in the next gap
};

// ---------------------------------------------------------------------------
// 2. BeamDetector
// ---------------------------------------------------------------------------
// Feed it the timestamp of every pulse the receiver module outputs, and call
// poll() regularly. A trigger is "no pulse for break_gap_us while the beam was
// present". The trigger timestamp comes from the LAST PULSE SEEN, not from when
// poll() noticed the gap, so it does not depend on how often poll() runs.
//
// States:
//   Searching - no steady beam yet (misaligned, blocked, just powered on)
//   Present   - beam is steady; a gap now counts as a trigger
//   Locked    - just triggered; ignoring everything for lockout_us, then back
//               to Searching so the beam has to be re-acquired before the next
//               trigger (this also means someone standing in the beam cannot
//               re-trigger it)
//
// Not thread-safe. On the ESP32, call onPulse() from the GPIO ISR and poll()
// from a task, both inside the same critical section (see receiver.cpp).
class BeamDetector {
public:
    enum State { Searching = 0, Present = 1, Locked = 2 };

    struct Event {
        bool triggered;
        int64_t t_us;  // device clock, microseconds
        Event() : triggered(false), t_us(0) {}
    };

    explicit BeamDetector(const Config& c = Config()) : cfg_(c) { reset(); }

    void reset() {
        state_ = Searching;
        have_pulse_ = false;
        last_pulse_ = 0;
        run_ = 0;
        total_ = 0;
        lock_until_ = 0;
        pending_ = false;
        pending_t_ = 0;
    }

    void onPulse(int64_t t) {
        ++total_;
        maybeUnlock(t);
        // If poll() was late and the beam came back, the gap is still a trigger.
        if (state_ == Present && (t - last_pulse_) > (int64_t)cfg_.break_gap_us) fire();
        if (state_ == Locked) return;

        if (!have_pulse_ || (t - last_pulse_) > (int64_t)cfg_.break_gap_us) run_ = 1;
        else ++run_;
        have_pulse_ = true;
        last_pulse_ = t;
        if (state_ == Searching && run_ >= cfg_.acquire_pulses) state_ = Present;
    }

    Event poll(int64_t now) {
        if (state_ == Present && (now - last_pulse_) > (int64_t)cfg_.break_gap_us) fire();
        maybeUnlock(now);
        Event e;
        if (pending_) {
            e.triggered = true;
            e.t_us = pending_t_;
            pending_ = false;
        }
        return e;
    }

    State state() const { return state_; }
    uint32_t totalPulses() const { return total_; }

private:
    void fire() {
        state_ = Locked;
        pending_ = true;
        pending_t_ = last_pulse_ + (int64_t)cfg_.trigger_offset_us;
        lock_until_ = last_pulse_ + (int64_t)cfg_.lockout_us;
    }
    void maybeUnlock(int64_t now) {
        if (state_ == Locked && now >= lock_until_) {
            state_ = Searching;
            have_pulse_ = false;
            run_ = 0;
        }
    }

    Config cfg_;
    State state_;
    bool have_pulse_;
    int64_t last_pulse_;
    uint32_t run_;
    uint32_t total_;
    int64_t lock_until_;
    bool pending_;
    int64_t pending_t_;
};

// ---------------------------------------------------------------------------
// 3. proto: BLE packets
// ---------------------------------------------------------------------------
// One BLE service per receiver, two characteristics:
//   CMD (write, no response):  phone -> receiver
//   EVT (notify):              receiver -> phone
// All multi-byte values are little-endian. Every packet fits in the default
// 20-byte BLE payload.
//
//   CMD  'A'                      arm
//        'D'                      disarm
//        'P' id:u32               ping; reply with a PONG
//        'R'                      reset detector
//
//   EVT  0x01 id:u32 t_us:i64     PONG   t_us = receiver clock when the ping arrived
//        0x02 seq:u16 t_us:i64    TRIGGER (receiver clock)
//        0x03 state:u8 armed:u8 rate_hz:u16 expected_hz:u16   STATUS
namespace proto {

enum Cmd { CMD_ARM = 'A', CMD_DISARM = 'D', CMD_PING = 'P', CMD_RESET = 'R' };
enum Evt { EVT_PONG = 0x01, EVT_TRIGGER = 0x02, EVT_STATUS = 0x03 };

static const size_t MAX_PACKET = 20;

inline void put_u16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}
inline void put_u32(uint8_t* p, uint32_t v) {
    for (int i = 0; i < 4; ++i) p[i] = (uint8_t)((v >> (8 * i)) & 0xFF);
}
inline void put_i64(uint8_t* p, int64_t v) {
    uint64_t u = (uint64_t)v;
    for (int i = 0; i < 8; ++i) p[i] = (uint8_t)((u >> (8 * i)) & 0xFF);
}
inline uint16_t get_u16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline uint32_t get_u32(const uint8_t* p) {
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i) v |= (uint32_t)p[i] << (8 * i);
    return v;
}
inline int64_t get_i64(const uint8_t* p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v |= (uint64_t)p[i] << (8 * i);
    return (int64_t)v;
}

inline size_t pack_pong(uint8_t* b, uint32_t id, int64_t t_us) {
    b[0] = EVT_PONG;
    put_u32(b + 1, id);
    put_i64(b + 5, t_us);
    return 13;
}
inline size_t pack_trigger(uint8_t* b, uint16_t seq, int64_t t_us) {
    b[0] = EVT_TRIGGER;
    put_u16(b + 1, seq);
    put_i64(b + 3, t_us);
    return 11;
}
inline size_t pack_status(uint8_t* b, uint8_t state, bool armed, uint16_t rate_hz, uint16_t expected_hz) {
    b[0] = EVT_STATUS;
    b[1] = state;
    b[2] = armed ? 1 : 0;
    put_u16(b + 3, rate_hz);
    put_u16(b + 5, expected_hz);
    return 7;
}

}  // namespace proto

// ---------------------------------------------------------------------------
// 4. Clock offset estimate (the phone app does this; kept here so it is tested)
// ---------------------------------------------------------------------------
// For each ping the phone records t_send and t_recv on its own clock, and the
// receiver replies with t_dev on the receiver's clock. Assuming the ping and
// the reply took equal time, the receiver stamped halfway through the round
// trip. The sample with the shortest round trip has the least room for
// asymmetry, so use that one.
//
//   offset = t_dev - (t_send + rtt/2)       => t_dev = t_phone + offset
//
// Convert a receiver timestamp to phone time with  t_phone = t_dev - offset.
struct SyncSample {
    int64_t t_send_us;
    int64_t t_recv_us;
    int64_t t_dev_us;
};
struct SyncResult {
    int64_t offset_us;
    int64_t rtt_us;
    bool ok;
    SyncResult() : offset_us(0), rtt_us(0), ok(false) {}
};

inline SyncResult estimate_offset(const SyncSample* s, size_t n) {
    SyncResult r;
    bool have = false;
    for (size_t i = 0; i < n; ++i) {
        int64_t rtt = s[i].t_recv_us - s[i].t_send_us;
        if (rtt < 0) continue;
        if (!have || rtt < r.rtt_us) {
            r.rtt_us = rtt;
            r.offset_us = s[i].t_dev_us - (s[i].t_send_us + rtt / 2);
            have = true;
        }
    }
    r.ok = have;
    return r;
}

}  // namespace gate
