#include "guidance.h"

#include <cmath>
#include <algorithm>
#include <chrono>

namespace trk {

static double fuzzy_tri(double x, double center, double width) {
    if (width <= 0) return (x == center) ? 1.0 : 0.0;
    return std::max(0.0, 1.0 - std::abs(x - center) / width);
}

double Guidance::fuzzy_control(double err, int deadband, double half, int max_out) const {
    if (std::abs(err) < deadband) return 0;
    double norm = std::max(-1.0, std::min(1.0, err / half));

    struct Mf { const char* name; double c, w; int out; };
    static const Mf mfs[] = {
        {"NL", -1.0, 0.5, -max_out},
        {"NS", -0.5, 0.4, -max_out / 2},
        {"Z",   0.0, 0.3,  0},
        {"PS",  0.5, 0.4,  max_out / 2},
        {"PL",  1.0, 0.5,  max_out},
    };
    double numer = 0, denom = 0;
    for (auto& m : mfs) {
        double mu = fuzzy_tri(norm, m.c, m.w);
        if (mu > 0) { numer += mu * m.out; denom += mu; }
    }
    if (denom == 0) return 0;
    return (int)std::lround(numer / denom);
}

void Guidance::reset() {
    have_att_ = false;
    prev_roll_ = prev_pitch_ = prev_yaw_ = 0;
}

void Guidance::update() {
    double half_w = cfg_.cam_w / 2.0;
    double half_h = cfg_.cam_h / 2.0;

    double cx, cy;
    TrackMode mode;
    bool have_track;
    std::array<int, 4> pilot;
    {
        std::lock_guard<std::mutex> lk(g.mtx_trk);
        cx = g.trk.center.x;
        cy = g.trk.center.y;
        mode = g.trk.mode;
        have_track = g.trk.active;
    }
    {
        std::lock_guard<std::mutex> lk(g.mtx_tlm);
        pilot = g.tlm.pilot;
    }

    int roll_off = 0, pitch_off = 0, yaw_off = 0;
    if (have_track && (mode == TrackMode::LOCK || mode == TrackMode::PREDICT)) {
        // автоподправка только когда пилот отпустил стики (roll/pitch/yaw)
        bool sticks_idle =
            std::abs(pilot[0] - cfg_.rc_neutral) <= cfg_.ap_stick_deadband &&
            std::abs(pilot[1] - cfg_.rc_neutral) <= cfg_.ap_stick_deadband &&
            std::abs(pilot[3] - cfg_.rc_neutral) <= cfg_.ap_stick_deadband;
        if (sticks_idle) {
            double err_y = cy - half_h;          // питч: цель выше центра → −
            double err_x = cx - half_w;          // рысканье: цель правее центра → +
            pitch_off = (int)fuzzy_control(-err_y, cfg_.ap_deadband, half_h, cfg_.ap_max_offset);
            yaw_off   = (int)fuzzy_control( err_x, cfg_.ap_deadband, half_w, cfg_.ap_max_offset);
            roll_off  = 0;                       // крен не задействуем — ведём рысканьем/питчем
        }
    } else if (mode == TrackMode::LOST) {
        // LOST: стоп коррекций и подхода, чистые оффсеты → пилоту
        std::lock_guard<std::mutex> lk(g.mtx_ovr);
        g.roll_off = 0; g.pitch_off = 0; g.yaw_off = 0;
        g.have_offsets = false;
        prev_roll_ = prev_pitch_ = prev_yaw_ = 0;
        return;
    }

    // демпфирование по угловой скорости FC
    auto now = std::chrono::steady_clock::now().time_since_epoch().count() * 1e-9;
    {
        std::lock_guard<std::mutex> lk(g.mtx_tlm);
        double fr = g.tlm.roll, fp = g.tlm.pitch;
        if (have_att_ && prev_att_time_ > 0) {
            double dt = now - prev_att_time_;
            if (dt > 0.001 && dt < 0.5) {
                pitch_off -= (int)(cfg_.ap_att_damp * (fp - prev_att_pitch_) / dt);
                yaw_off   -= (int)(cfg_.ap_att_damp * (fr - prev_att_roll_) / dt);
            }
        }
        prev_att_roll_ = fr;
        prev_att_pitch_ = fp;
        prev_att_time_ = now;
        have_att_ = true;
    }

    // ограничения + slew-rate (плавность)
    int max = cfg_.ap_max_offset;
    pitch_off = std::max(-max, std::min(max, pitch_off));
    yaw_off   = std::max(-max, std::min(max, yaw_off));

    int slew = (int)cfg_.ap_slew_rate;
    pitch_off = std::max(prev_pitch_ - slew, std::min(prev_pitch_ + slew, pitch_off));
    yaw_off   = std::max(prev_yaw_   - slew, std::min(prev_yaw_   + slew, yaw_off));

    prev_roll_  = roll_off;
    prev_pitch_ = pitch_off;
    prev_yaw_   = yaw_off;

    {
        std::lock_guard<std::mutex> lk(g.mtx_ovr);
        g.roll_off = roll_off;
        g.pitch_off = pitch_off;
        g.yaw_off = yaw_off;
        g.have_offsets = true;
    }
}

} // namespace trk