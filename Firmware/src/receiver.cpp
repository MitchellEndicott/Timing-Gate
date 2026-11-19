// receiver.cpp: IR receiver post with BLE.
//
// The TSOP38238 output goes low for each IR burst. A GPIO interrupt timestamps
// every falling edge with the free-running 64-bit microsecond clock. A missing
// run of pulses is the trigger (see gate_core.h), and the trigger timestamp is
// the last pulse seen, so it does not depend on loop timing or BLE traffic.
//
// Wiring (unverified until you test it):
//   TSOP38238 OUT -> D2 (GPIO4), VS -> 3V3 (with the datasheet's supply filter), GND -> GND
//
// Phone interface: see gate_core.h, namespace proto.
// Bench interface (no phone needed): open the serial monitor and type
//   a = arm, d = disarm, r = reset, s = status

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "esp_timer.h"
#include "gate_core.h"

using namespace gate;

// ---- pins and identifiers -------------------------------------------------
static const int RX_PIN = D2;
static const char* SVC_UUID = "7a1c0001-6b1e-4c1d-9d3a-5f6e7d8c9b01";
static const char* CMD_UUID = "7a1c0002-6b1e-4c1d-9d3a-5f6e7d8c9b01";
static const char* EVT_UUID = "7a1c0003-6b1e-4c1d-9d3a-5f6e7d8c9b01";

// After a trigger, disarm automatically. One trigger per Arm tap.
static const bool ONE_SHOT_PER_ARM = true;

// ---- shared state ---------------------------------------------------------
static Config cfg;
static BeamDetector det(cfg);
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool armed = false;
static uint16_t trigger_seq = 0;

// Outgoing BLE packets are queued, then sent from loop(). That keeps BLE calls
// out of the NimBLE callback context. Timestamps are taken BEFORE queueing.
struct Packet {
    uint8_t len;
    uint8_t data[proto::MAX_PACKET];
};
static const int QN = 16;
static Packet q[QN];
static volatile int q_head = 0, q_tail = 0;

static void q_push(const uint8_t* d, size_t len) {
    portENTER_CRITICAL(&mux);
    int next = (q_head + 1) % QN;
    if (next != q_tail) {  // drop if full rather than block
        q[q_head].len = (uint8_t)len;
        memcpy(q[q_head].data, d, len);
        q_head = next;
    }
    portEXIT_CRITICAL(&mux);
}
static bool q_pop(Packet& out) {
    bool got = false;
    portENTER_CRITICAL(&mux);
    if (q_tail != q_head) {
        out = q[q_tail];
        q_tail = (q_tail + 1) % QN;
        got = true;
    }
    portEXIT_CRITICAL(&mux);
    return got;
}

// ---- pulse interrupt ------------------------------------------------------
static void IRAM_ATTR on_pulse_isr() {
    int64_t t = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&mux);
    det.onPulse(t);
    portEXIT_CRITICAL_ISR(&mux);
}

// ---- commands -------------------------------------------------------------
static void do_arm(bool on) {
    armed = on;
    Serial.println(on ? "armed" : "disarmed");
}

static void handle_command(const uint8_t* d, size_t n, int64_t t_arrival) {
    if (n < 1) return;
    switch (d[0]) {
        case proto::CMD_ARM:
            do_arm(true);
            break;
        case proto::CMD_DISARM:
            do_arm(false);
            break;
        case proto::CMD_RESET:
            portENTER_CRITICAL(&mux);
            det.reset();
            portEXIT_CRITICAL(&mux);
            break;
        case proto::CMD_PING:
            if (n >= 5) {
                uint8_t b[proto::MAX_PACKET];
                size_t len = proto::pack_pong(b, proto::get_u32(d + 1), t_arrival);
                q_push(b, len);
            }
            break;
    }
}

// ---- BLE ------------------------------------------------------------------
static NimBLECharacteristic* evtChar = nullptr;
static volatile bool connected = false;

class ServerCb : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s) override {
        connected = true;
        Serial.println("phone connected");
    }
    void onDisconnect(NimBLEServer* s) override {
        connected = false;
        Serial.println("phone disconnected");
        NimBLEDevice::startAdvertising();
    }
};

class CmdCb : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c) override {
        int64_t t = esp_timer_get_time();  // stamp first, parse after
        std::string v = c->getValue();
        handle_command((const uint8_t*)v.data(), v.size(), t);
    }
};

static void ble_setup() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    char name[24];
    snprintf(name, sizeof(name), "gate-rx-%02X%02X", mac[4], mac[5]);

    NimBLEDevice::init(name);
    NimBLEServer* server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCb());
    NimBLEService* svc = server->createService(SVC_UUID);
    NimBLECharacteristic* cmd =
        svc->createCharacteristic(CMD_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    cmd->setCallbacks(new CmdCb());
    evtChar = svc->createCharacteristic(EVT_UUID, NIMBLE_PROPERTY::NOTIFY);
    svc->start();

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(SVC_UUID);
    adv->setScanResponse(true);
    adv->start();
    Serial.printf("BLE advertising as %s\n", name);
}

static void flush_ble() {
    Packet p;
    while (q_pop(p)) {
        if (connected && evtChar) {
            evtChar->setValue(p.data, p.len);
            evtChar->notify();
        }
    }
}

// ---- status ---------------------------------------------------------------
static uint32_t last_status_ms = 0;
static uint32_t last_total = 0;

static void send_status() {
    uint32_t total;
    BeamDetector::State st;
    portENTER_CRITICAL(&mux);
    total = det.totalPulses();
    st = det.state();
    portEXIT_CRITICAL(&mux);

    uint32_t now_ms = millis();
    uint32_t dt = now_ms - last_status_ms;
    uint16_t rate = dt ? (uint16_t)(((total - last_total) * 1000ULL) / dt) : 0;
    uint16_t expected = (uint16_t)(1000000UL / cfg.burst_period_us);
    last_status_ms = now_ms;
    last_total = total;

    uint8_t b[proto::MAX_PACKET];
    size_t len = proto::pack_status(b, (uint8_t)st, armed, rate, expected);
    q_push(b, len);

    static const char* names[] = {"searching", "present", "locked"};
    Serial.printf("beam=%s armed=%d pulses/s=%u (expect %u)\n", names[st], (int)armed, rate, expected);
}

// ---- Arduino entry points ------------------------------------------------
void setup() {
    Serial.begin(115200);
    pinMode(RX_PIN, INPUT);  // TSOP output is push-pull, idle high
    attachInterrupt(digitalPinToInterrupt(RX_PIN), on_pulse_isr, FALLING);
    ble_setup();
    Serial.println("receiver ready. serial: a=arm d=disarm r=reset s=status");
}

void loop() {
    // 1. Look for a trigger.
    BeamDetector::Event e;
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&mux);
    e = det.poll(now);
    portEXIT_CRITICAL(&mux);

    if (e.triggered) {
        if (armed) {
            uint8_t b[proto::MAX_PACKET];
            size_t len = proto::pack_trigger(b, ++trigger_seq, e.t_us);
            q_push(b, len);
            Serial.printf("TRIGGER #%u at %lld us\n", (unsigned)trigger_seq, (long long)e.t_us);
            if (ONE_SHOT_PER_ARM) armed = false;
        } else {
            Serial.println("beam broken (not armed, ignored)");
        }
    }

    // 2. Status about twice a second.
    if (millis() - last_status_ms >= 500) send_status();

    // 3. Bench commands from the serial monitor.
    while (Serial.available()) {
        char ch = (char)Serial.read();
        if (ch == 'a') do_arm(true);
        else if (ch == 'd') do_arm(false);
        else if (ch == 'r') {
            portENTER_CRITICAL(&mux);
            det.reset();
            portEXIT_CRITICAL(&mux);
            Serial.println("reset");
        } else if (ch == 's') send_status();
    }

    flush_ble();
    delay(1);
}
