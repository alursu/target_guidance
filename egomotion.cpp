#include "egomotion.h"

#include <cmath>

namespace trk {

// Малая-угловая модель: изображение смещается на f·Δangle.
//  pitch-вращение → вертикаль, yaw-вращение → горизонталь.
// Ролик (вращение вокруг оптической оси) при малых углах и цели в центре
// даёт пренебрежимо малый сдвиг — игнорируем.
EgoShift ego_from_attitude(const Telemetry& prev, const Telemetry& curr, const Config& cfg) {
    EgoShift s;
    double focal = (cfg.cam_w / 2.0) / std::tan(cfg.cam_hfov * M_PI / 180.0 / 2.0);

    double d_pitch = curr.pitch - prev.pitch;   // град
    double d_yaw   = curr.yaw   - prev.yaw;

    // «свежесть»: нулевая телеметрия (нет данных с FC) — флаг снять
    if (fabs(curr.pitch) + fabs(curr.roll) + fabs(curr.yaw) < 0.01) return s;

    s.dx = focal * d_yaw   * M_PI / 180.0;
    s.dy = focal * d_pitch * M_PI / 180.0;
    s.valid = true;
    return s;
}

} // namespace trk