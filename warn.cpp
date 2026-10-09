#include "warn.h"

#include <thread>
#include <chrono>
#include <unistd.h>
#include <cstdio>

namespace trk {

bool Warn::start() {
    if (cfg_.gpio_pin < 0) {
        fprintf(stderr, "[WARN] зуммер отключён (gpio_pin<0)\n");
        return false;
    }
    gpio_ = cfg_.gpio_pin;
    // проверяем доступность gpioset
    char probe[128];
    snprintf(probe, sizeof probe, "gpioset %s %d=0 >/dev/null 2>&1",
             cfg_.gpio_chip.c_str(), gpio_);
    if (system(probe) != 0) {
        fprintf(stderr, "[WARN] gpioset недоступен для %s p%d — зуммер выкл.\n",
                cfg_.gpio_chip.c_str(), gpio_);
        return false;
    }
    running_ = true;
    std::thread(&Warn::loop, this).detach();
    fprintf(stderr, "[WARN] зуммер: %s p%d\n", cfg_.gpio_chip.c_str(), gpio_);
    return true;
}

void Warn::stop() {
    running_ = false;
    if (gpio_ >= 0) {
        char off[128];
        snprintf(off, sizeof off, "gpioset %s %d=0 >/dev/null 2>&1",
                 cfg_.gpio_chip.c_str(), gpio_);
        system(off);
    }
}

void Warn::loop() {
    bool prev = false;
    while (running_) {
        bool lost = g.show_lost.load();
        if (lost != prev) {
            char cmd[128];
            snprintf(cmd, sizeof cmd, "gpioset %s %d=%d >/dev/null 2>&1",
                     cfg_.gpio_chip.c_str(), gpio_, lost ? 1 : 0);
            system(cmd);
            prev = lost;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

} // namespace trk