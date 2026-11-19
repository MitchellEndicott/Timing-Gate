// Host-side tests for gate_core.h. No hardware needed.
//   make -C host_tests        (or see the Makefile)

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "gate_core.h"

using namespace gate;

static int failures = 0;
#define CHECK(cond, msg)                                                  \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::printf("  FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__); \
            ++failures;                                                   \
        }                                                                 \
    } while (0)

// Deterministic RNG so results repeat.
static uint64_t rng_state = 88172645463325252ULL;
static uint64_t rnd() {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return rng_state;
}
static int64_t rnd_range(int64_t lo, int64_t hi) { return lo + (int64_t)(rnd() % (uint64_t)(hi - lo + 1)); }

// ---------------------------------------------------------------------------
// Beam simulator: an emitter pulsing every period, with intervals where the
// receiver sees nothing (beam blocked). Time steps of 50 us; poll every 1 ms.
// ---------------------------------------------------------------------------
struct Block {
    int64_t from, to;
};
struct Trig {
    int64_t t_us;
    int64_t polled_at;
};

static std::vector<Trig> simulate(BeamDetector& det, const Config& cfg, int64_t end_us,
                                  const std::vector<Block>& blocks, int64_t poll_every_us = 1000,
                                  int64_t skip_poll_from = -1, int64_t skip_poll_to = -1) {
    std::vector<Trig> out;
    int64_t next_pulse = 500;  // arbitrary phase
    int64_t next_poll = poll_every_us;
    for (int64_t t = 0; t <= end_us; t += 50) {
        if (t >= next_pulse) {
            bool blocked = false;
            for (size_t i = 0; i < blocks.size(); ++i)
                if (next_pulse >= blocks[i].from && next_pulse < blocks[i].to) blocked = true;
            if (!blocked) det.onPulse(next_pulse);
            next_pulse += cfg.burst_period_us;
        }
        if (t >= next_poll) {
            next_poll += poll_every_us;
            if (skip_poll_from >= 0 && t >= skip_poll_from && t < skip_poll_to) continue;
            BeamDetector::Event e = det.poll(t);
            if (e.triggered) {
                Trig tr = {e.t_us, t};
                out.push_back(tr);
            }
        }
    }
    return out;
}

static int64_t iabs(int64_t v) { return v < 0 ? -v : v; }

// ---------------------------------------------------------------------------
static void test_basic_trigger() {
    std::printf("basic trigger and timestamp error\n");
    Config cfg;
    double sum_abs = 0;
    int64_t worst = 0;
    const int N = 500;
    for (int i = 0; i < N; ++i) {
        BeamDetector det(cfg);
        int64_t t_break = 100000 + rnd_range(0, 1199);  // random phase against the pulse train
        std::vector<Block> b;
        Block bl = {t_break, 400000};
        b.push_back(bl);
        std::vector<Trig> tr = simulate(det, cfg, 300000, b);
        CHECK(tr.size() == 1, "exactly one trigger");
        if (tr.size() == 1) {
            int64_t err = tr[0].t_us - t_break;
            sum_abs += (double)iabs(err);
            worst = std::max(worst, iabs(err));
            CHECK(iabs(err) <= (int64_t)cfg.burst_period_us / 2 + 50, "timestamp within half a period");
            // Detection latency: how long after the break did poll() notice?
            CHECK(tr[0].polled_at - t_break < (int64_t)(cfg.break_gap_us + cfg.burst_period_us + 2000),
                  "detected within gap + period + poll slop");
        }
    }
    std::printf("  timestamp error: mean %.0f us, worst %lld us over %d trials\n", sum_abs / N,
                (long long)worst, N);
}

static void test_dropouts() {
    std::printf("short dropouts must not trigger, long ones must\n");
    Config cfg;
    {
        BeamDetector det(cfg);
        // Remove 3 consecutive pulses (3.6 ms) while the beam is present.
        std::vector<Block> b;
        Block bl = {100000, 100000 + 3 * 1200 - 100};
        b.push_back(bl);
        CHECK(simulate(det, cfg, 300000, b).empty(), "3 missed pulses: no trigger");
    }
    {
        BeamDetector det(cfg);
        std::vector<Block> b;
        Block bl = {100000, 100000 + 6 * 1200};
        b.push_back(bl);
        // Beam returns afterwards, but the trigger has already fired.
        CHECK(simulate(det, cfg, 300000, b).size() == 1, "6 missed pulses: trigger");
    }
    std::printf("  threshold is break_gap_us = %u us (%u missed pulses)\n", (unsigned)cfg.break_gap_us,
                (unsigned)(cfg.break_gap_us / cfg.burst_period_us));
}

static void test_lockout_and_reacquire() {
    std::printf("lockout, re-acquire, and a person standing in the beam\n");
    Config cfg;
    BeamDetector det(cfg);
    std::vector<Block> b;
    Block b1 = {50000, 80000};      // first break, 30 ms
    Block b2 = {300000, 330000};    // flicker 250 ms later: inside lockout
    Block b3 = {1500000, 1600000};  // real second break after lockout and re-acquire
    b.push_back(b1);
    b.push_back(b2);
    b.push_back(b3);
    std::vector<Trig> tr = simulate(det, cfg, 3000000, b);
    CHECK(tr.size() == 2, "two triggers total (flicker ignored)");

    // Beam blocked for the whole run: Present is never reached, so no trigger.
    BeamDetector det2(cfg);
    std::vector<Block> all;
    Block bl = {0, 10000000};
    all.push_back(bl);
    CHECK(simulate(det2, cfg, 1000000, all).empty(), "blocked from power-up: no trigger");
    CHECK(det2.state() == BeamDetector::Searching, "stays Searching");

    // Beam appears late: needs acquire_pulses before it counts.
    BeamDetector det3(cfg);
    std::vector<Block> late;
    Block lb = {0, 100000};
    late.push_back(lb);
    simulate(det3, cfg, 130000, late);
    CHECK(det3.state() == BeamDetector::Present, "late beam acquired");
}

static void test_late_poll() {
    std::printf("poll() stalled during the gap still gives the right timestamp\n");
    Config cfg;
    BeamDetector ref(cfg), stalled(cfg);
    std::vector<Block> b;
    Block bl = {100000, 130000};
    b.push_back(bl);
    std::vector<Trig> a = simulate(ref, cfg, 300000, b);
    std::vector<Trig> c = simulate(stalled, cfg, 300000, b, 1000, 95000, 140000);
    CHECK(a.size() == 1 && c.size() == 1, "both trigger once");
    if (a.size() == 1 && c.size() == 1) CHECK(a[0].t_us == c[0].t_us, "same timestamp regardless of poll timing");
}

static void test_protocol() {
    std::printf("packet round trips\n");
    uint8_t b[proto::MAX_PACKET];
    size_t n = proto::pack_pong(b, 0xA1B2C3D4u, 123456789012345LL);
    CHECK(n == 13 && b[0] == proto::EVT_PONG, "pong header");
    CHECK(proto::get_u32(b + 1) == 0xA1B2C3D4u, "pong id");
    CHECK(proto::get_i64(b + 5) == 123456789012345LL, "pong time");
    n = proto::pack_trigger(b, 513, -42);
    CHECK(n == 11 && proto::get_u16(b + 1) == 513 && proto::get_i64(b + 3) == -42, "trigger");
    n = proto::pack_status(b, 1, true, 830, 833);
    CHECK(n == 7 && b[1] == 1 && b[2] == 1 && proto::get_u16(b + 3) == 830 && proto::get_u16(b + 5) == 833,
          "status");
}

// ---------------------------------------------------------------------------
// Clock sync: model BLE as random one-way delays. Real BLE delays are
// quantized by the connection interval, so the delay range here is an
// assumption. Change it to match what you measure.
// ---------------------------------------------------------------------------
struct SyncStats {
    double median, p95;
    int64_t worst;
};
static SyncStats sync_error_stats(int pings, int64_t dmin, int64_t dmax, int trials,
                                  std::vector<int64_t>* errs_out) {
    std::vector<int64_t> errs;
    for (int k = 0; k < trials; ++k) {
        int64_t true_offset = rnd_range(-5000000000LL, 5000000000LL);  // device clock vs phone clock
        std::vector<SyncSample> s;
        int64_t t = 1000000;
        for (int i = 0; i < pings; ++i) {
            int64_t up = rnd_range(dmin, dmax), down = rnd_range(dmin, dmax);
            SyncSample x;
            x.t_send_us = t;
            x.t_dev_us = (t + up) + true_offset;  // receiver stamps on arrival
            x.t_recv_us = t + up + down;
            s.push_back(x);
            t += 50000;
        }
        SyncResult r = estimate_offset(&s[0], s.size());
        errs.push_back(r.offset_us - true_offset);
    }
    std::vector<int64_t> a;
    int64_t worst = 0;
    for (size_t i = 0; i < errs.size(); ++i) {
        a.push_back(iabs(errs[i]));
        worst = std::max(worst, iabs(errs[i]));
    }
    std::sort(a.begin(), a.end());
    SyncStats st = {(double)a[a.size() / 2], (double)a[(size_t)(a.size() * 0.95)], worst};
    if (errs_out) *errs_out = errs;
    return st;
}

static void test_sync() {
    std::printf("clock sync error over simulated BLE delays\n");
    const int64_t ranges[][2] = {{2000, 15000}, {5000, 12000}, {7500, 7500}};
    const char* labels[] = {"2-15 ms one-way, uniform", "5-12 ms one-way, uniform", "7.5 ms fixed (ideal)"};
    for (int r = 0; r < 3; ++r) {
        for (int pings = 5; pings <= 20; pings *= 2) {
            SyncStats st = sync_error_stats(pings, ranges[r][0], ranges[r][1], 3000, 0);
            std::printf("  %-26s %2d pings: median %6.0f us, p95 %6.0f us, worst %6lld us\n", labels[r], pings,
                        st.median, st.p95, (long long)st.worst);
            int64_t bound = (ranges[r][1] - ranges[r][0]) / 2;
            CHECK(st.worst <= bound + 1, "error never exceeds half the delay asymmetry range");
        }
    }

    // Two posts, one physical event: split-time error is the difference of two sync errors.
    std::vector<int64_t> ea, eb;
    sync_error_stats(10, 2000, 15000, 3000, &ea);
    sync_error_stats(10, 2000, 15000, 3000, &eb);
    std::vector<int64_t> diff;
    for (size_t i = 0; i < ea.size(); ++i) diff.push_back(iabs(ea[i] - eb[i]));
    std::sort(diff.begin(), diff.end());
    std::printf("  two posts, same beam, 10 pings each (2-15 ms): median %lld us, p95 %lld us, worst %lld us\n",
                (long long)diff[diff.size() / 2], (long long)diff[(size_t)(diff.size() * 0.95)],
                (long long)diff.back());
}

int main() {
    test_basic_trigger();
    test_dropouts();
    test_lockout_and_reacquire();
    test_late_poll();
    test_protocol();
    test_sync();
    if (failures) {
        std::printf("\n%d check(s) FAILED\n", failures);
        return 1;
    }
    std::printf("\nall checks passed\n");
    return 0;
}
