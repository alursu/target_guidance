#pragma once

#include "common.h"

namespace trk {

// ─── Предупреждение пилоту ──────────────────────────────────────────────
//  * мигающий OSD-баннер рисуется в osd.cpp (g.show_lost)
//  * зуммер на GPIO (gpioset <chip> <pin>=1/0) — фоновый поток
class Warn {
public:
    Warn(Config* cfg) : cfg_(*cfg) {}

    bool start();
    void stop();

private:
    Config& cfg_;
    std::atomic<bool> running_{false};
    int gpio_ = 0;
    void loop();
};

} // namespace trk