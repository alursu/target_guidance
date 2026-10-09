#pragma once

#include "common.h"
#include <array>

namespace trk {

// ─── Подруливание: ошибка «центр цели − центр кадра» → оффсеты каналов ──
// Имеет внутреннее состояние (мёртвая зона, slew-rate, демпфирование по
// угловой скорости FC). Вызывается из потоков реле (crsf_bridge).
class Guidance {
public:
    Guidance(Config* cfg) : cfg_(*cfg) {}

    // Пересчитывает оффсеты по текущему состоянию трекера и кладёт в Shared.
    // Вызывается, когда CH5 высок (режим сопровождения).
    void update();

    // вызывается при переходе engaged → true (сброс демпфирования)
    void reset();

private:
    Config& cfg_;

    int prev_roll_ = 0, prev_pitch_ = 0, prev_yaw_ = 0;
    double prev_att_roll_ = 0, prev_att_pitch_ = 0;
    double prev_att_time_ = 0;
    bool have_att_ = false;

    double fuzzy_control(double err, int deadband, double half, int max_out) const;
};

} // namespace trk