#pragma once

#include "common.h"

namespace trk {

// ─── Компенсация собственного движения камеры ──────────────────────────
// Сдвиг кадра в пикселях, вызванный поворотом дрона между двумя кадрами.
struct EgoShift {
    double dx = 0, dy = 0;   // сдвиг в пикселях (питч→вертикаль, рысканье→горизонталь)
    bool valid = false;      // true, если углы FC свежие (можно опираться)
};

EgoShift ego_from_attitude(const Telemetry& prev, const Telemetry& curr, const Config& cfg);

} // namespace trk