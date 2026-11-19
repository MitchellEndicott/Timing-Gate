// Prints the packets gate_core.h produces, as hex, so test_page_core.js can
// check that the phone page parses exactly what the firmware sends.
#include <cstdio>
#include "gate_core.h"

static void hex(const char* name, const uint8_t* b, size_t n) {
    std::printf("%s ", name);
    for (size_t i = 0; i < n; ++i) std::printf("%02x", b[i]);
    std::printf("\n");
}

int main() {
    uint8_t b[gate::proto::MAX_PACKET];
    size_t n;
    n = gate::proto::pack_pong(b, 0xA1B2C3D4u, 123456789012345LL);
    hex("pong", b, n);
    n = gate::proto::pack_trigger(b, 513, 9876543210LL);
    hex("trigger", b, n);
    n = gate::proto::pack_status(b, 1, true, 830, 833);
    hex("status", b, n);
    return 0;
}
