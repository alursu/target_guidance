#include "osd.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace trk {

namespace {
using cv::Mat;
using cv::Scalar;

const int FONT = cv::FONT_HERSHEY_SIMPLEX;
const Scalar WHT(255, 255, 255), CYN(255, 255, 0), GRN(0, 255, 0),
              RED(0, 0, 255), YLW(0, 255, 255), HOR(0, 255, 0);

struct Ref {
    int disp_w, disp_h;
    double camw, camh;
    double s;               // масштаб
};

void put_text(Mat& f, const std::string& t, int x, int y, const Scalar& c,
              double scale, int thick) {
    cv::putText(f, t, cv::Point(x, y), FONT, scale, Scalar(0, 0, 0), thick + 2, cv::LINE_AA);
    cv::putText(f, t, cv::Point(x, y), FONT, scale, c, thick, cv::LINE_AA);
}

void draw_crosshair(Mat& f, int cx, int cy, double s) {
    int arm = (int)(12 * s), gap = (int)(4 * s);
    cv::line(f, cv::Point(cx - arm, cy), cv::Point(cx - gap, cy), CYN, 1);
    cv::line(f, cv::Point(cx + gap, cy), cv::Point(cx + arm, cy), CYN, 1);
    cv::line(f, cv::Point(cx, cy - arm), cv::Point(cx, cy - gap), CYN, 1);
    cv::line(f, cv::Point(cx, cy + gap), cv::Point(cx, cy + arm), CYN, 1);
}

void draw_horizon(Mat& f, double roll_deg, double pitch_deg, float hfov, double s) {
    int h = f.rows, w = f.cols;
    int cx = w / 2, cy = h / 2;
    double roll = roll_deg * M_PI / 180.0, pitch = pitch_deg * M_PI / 180.0;
    double focal = w / (2.0 * std::tan(hfov * M_PI / 180.0 / 2.0));
    double pitch_px = focal * std::tan(pitch);
    pitch_px = std::max(-h * 0.45, std::min(h * 0.45, pitch_px));

    double cosr = std::cos(roll), sinr = std::sin(roll);
    int diag = (int)std::sqrt((double)w * w + (double)h * h);
    int dx = (int)(diag * cosr), dy = (int)(diag * sinr);
    cv::line(f, cv::Point(cx - dx, (int)(cy + pitch_px) - dy),
             cv::Point(cx + dx, (int)(cy + pitch_px) + dy), HOR, 2);

    for (int step = 10; step <= 80; step += 10) {
        for (int sign : {1, -1}) {
            double sr = sign * step * M_PI / 180.0;
            double y_off = focal * (std::tan(pitch + sr) - std::tan(pitch));
            if (std::abs(y_off) > h * 0.6) continue;
            int half_w = (int)(w * 0.15 * (1 - step / 90.0));
            int sx1 = (int)(cx - half_w * cosr), sy1 = (int)(cy + y_off - half_w * sinr);
            int sx2 = (int)(cx + half_w * cosr), sy2 = (int)(cy + y_off + half_w * sinr);
            Scalar c = (step % 20 == 0) ? WHT : Scalar(200, 200, 200);
            cv::line(f, cv::Point(sx1, sy1), cv::Point(sx2, sy2), c, 1);
            char buf[8];
            snprintf(buf, sizeof buf, "%s%d", sign < 0 ? "-" : "", step);
            put_text(f, buf, sx2 + (int)(4 * s), (int)(cy + y_off - 6 * cosr), c, 0.4 * s, 1);
        }
    }
    int gap = (int)(w * 0.25), vlen = (int)(h * 0.35);
    for (int step_h : {-gap, gap}) {
        int hx = (int)(cx + step_h * cosr), hy = (int)(cy + pitch_px + step_h * sinr);
        cv::line(f, cv::Point(hx - (int)(vlen * sinr), hy + (int)(vlen * cosr)),
                 cv::Point(hx + (int)(vlen * sinr), hy - (int)(vlen * cosr)),
                 Scalar(120, 120, 120), 1);
    }
}

std::string mode_str(TrackMode m) {
    switch (m) {
        case TrackMode::MANUAL:  return "MANUAL";
        case TrackMode::LOCK:    return "LOCK";
        case TrackMode::PREDICT: return "PREDICT";
        case TrackMode::LOST:    return "LOST";
    }
    return "?";
}

} // namespace

Osd::Osd(Config* cfg) : cfg_(*cfg) {}

void Osd::open_window() {
    cv::namedWindow("OSD", cv::WINDOW_NORMAL);
    if (cfg_.fullscreen)
        cv::setWindowProperty("OSD", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
}

void Osd::draw(Mat& frame_out, const Mat& raw, double now, uint32_t frame_no) {
    int w = frame_out.cols, h = frame_out.rows;
    double s = std::min((double)w / cfg_.disp_w, (double)h / cfg_.disp_h);

    Telemetry tlm;
    TrackState trk;
    bool show_lost;
    {
        std::lock_guard<std::mutex> lk(g.mtx_tlm);
        tlm = g.tlm;
    }
    {
        std::lock_guard<std::mutex> lk(g.mtx_trk);
        trk = g.trk;
    }
    show_lost = g.show_lost.load();

    // положение цели в дисплейных координатах (по фактическому размеру кадра)
    double sx = (double)w / std::max(raw.cols, 1), sy = (double)h / std::max(raw.rows, 1);
    if (trk.active && trk.roi.width > 0) {
        cv::Rect r((int)(trk.roi.x * sx), (int)(trk.roi.y * sy),
                   (int)(trk.roi.width * sx), (int)(trk.roi.height * sy));
        Scalar col = (trk.mode == TrackMode::LOCK) ? GRN : YLW;
        if (trk.mode == TrackMode::LOST) col = RED;
        cv::rectangle(frame_out, r, col, 2);

        cv::Point c((int)(trk.center.x * sx), (int)(trk.center.y * sy));
        cv::circle(frame_out, c, 5, RED, -1);
        cv::line(frame_out, cv::Point(w / 2, h / 2), c, Scalar(255, 0, 0), 1);

        // скорость движения цели
        put_text(frame_out, cv::format("vx=%.0f vy=%.0f", trk.vel.x, trk.vel.y),
                 c.x + 12, c.y, WHT, 0.4 * s, 1);
    }

    // верхний бар
    int mg = (int)(12 * s);
    double fs1 = 0.6 * s, fs2 = 0.5 * s;
    std::string armed = (tlm.ch6 > 700) ? "ARMED" : "DISARMED";
    Scalar ac = tlm.armed ? GRN : Scalar(0, 255, 255);
    put_text(frame_out, "FC: " + armed, mg, mg + (int)(18 * s), ac, fs1, 1);
    put_text(frame_out, "mode=" + mode_str(trk.mode), mg, mg + (int)(40 * s), CYN, fs2, 1);

    // центр-перекрестье + линия ошибки
    draw_crosshair(frame_out, w / 2, h / 2, s);
    draw_horizon(frame_out, tlm.roll, tlm.pitch, cfg_.cam_hfov, s);

    // телеметрия
    std::string hdg = cv::format("HDG %d", ((int)tlm.heading % 360 + 360) % 360);
    cv::Size ts = cv::getTextSize(hdg, FONT, fs2, 1, nullptr);
    put_text(frame_out, hdg, (w - ts.width) / 2, mg + (int)(18 * s), CYN, fs2, 1);

    std::string bat = cv::format("BAT %.2fV %d%%", tlm.vcc, tlm.bat_pct);
    ts = cv::getTextSize(bat, FONT, fs2, 1, nullptr);
    put_text(frame_out, bat, w - ts.width - mg, mg + (int)(18 * s), GRN, fs2, 1);

    std::string sat = cv::format("SAT %d  FIX %d", tlm.sats, tlm.fix);
    ts = cv::getTextSize(sat, FONT, fs2, 1, nullptr);
    put_text(frame_out, sat, w - ts.width - mg, mg + (int)(40 * s), CYN, fs2, 1);

    int y1 = h - mg - (int)(68 * s), y2 = h - mg - (int)(44 * s);
    put_text(frame_out, cv::format("ALT %.1fm", tlm.alt), mg, y1, CYN, fs2, 1);
    Scalar cc = (tlm.climb >= 0) ? GRN : RED;
    put_text(frame_out, cv::format("CLB %+.1fm/s", tlm.climb), mg, y2, cc, fs2, 1);

    std::string spd = (tlm.speed > 0) ? cv::format("SPD %.1fkm/h", tlm.speed) : "SPD --";
    ts = cv::getTextSize(spd, FONT, fs2, 1, nullptr);
    put_text(frame_out, spd, w - ts.width - mg, y1, CYN, fs2, 1);

    // живые каналы с пульта (для проверки RF-линка):
    // видно, что ELRS реально отдаёт кадры; CH5 — переключатель захвата
    Scalar eng = (tlm.ch5 > cfg_.rc_engage_threshold) ? GRN : RED;
    std::string rcs = cv::format("R=%d P=%d T=%d Y=%d  CH5=%d%s  CH6=%d",
                                 tlm.pilot[0], tlm.pilot[1], tlm.pilot[2], tlm.pilot[3],
                                 tlm.ch5, (tlm.ch5 > cfg_.rc_engage_threshold) ? " ENG" : "",
                                 tlm.ch6);
    put_text(frame_out, rcs, mg, h - mg - (int)(22 * s), eng, fs2, 1);

    // таймаут LOST → предупреждение
    if (trk.mode == TrackMode::LOST) g.show_lost.store(true);
    else if (trk.mode != TrackMode::PREDICT) g.show_lost.store(false);
    else if (!show_lost) g.show_lost.store(false);

    if (g.show_lost.load() && (frame_no / 12) % 2 == 0) {
        std::string msg = "!!! TARGET LOST - MANUAL !!!";
        cv::Size ts2 = cv::getTextSize(msg, FONT, 1.2 * s, 3, nullptr);
        put_text(frame_out, msg, (w - ts2.width) / 2, h / 2 - (int)(80 * s),
                 Scalar(0, 0, 255), 1.2 * s, 3);
    }
}

} // namespace trk