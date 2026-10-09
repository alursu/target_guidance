#pragma once

#include <string>
#include <vector>
#include <array>
#include <mutex>
#include <atomic>
#include <fstream>
#include <sstream>
#include <opencv2/core.hpp>

namespace trk {

// ─── Конфигурация (читается из tracker.ini) ───────────────────────────────
struct Config {
    // UART / CRSF
    std::string elrs_port = "/dev/ttyAMA0";
    std::string fc_port   = "/dev/ttyAMA2";
    int elrs_baud = 420000;     // фактическая скорость ELRS-приёмника
    int fc_baud   = 420000;     // CRSF на FC = стандартные 420000 (как у ELRS)
    bool enable_crsf = true;

    // Камера
    int camera_dev = 0;
    std::string camera_type = "v4l2";   // "v4l2" | "guide"
    int cam_w = 640, cam_h = 480, cam_fps = 25;
    int disp_w = 1920, disp_h = 1080;
    bool fullscreen = true;

    // Захват объекта (квадрат А: ровно один объект в квадрате)
    int acquire_box = 64;           // сторона квадрата в центре кадра, px
    int rc_neutral = 992;           // нейтраль канала
    int rc_engage_threshold = 1350; // CH5 > этого → захват / автосопровождение

    // Слежение
    int track_lost_frames = 6;      // столько кадров без успеха → поиск по движению
    double lost_timeout_s = 2.5;    // общий лимит «пропажи» до LOST
    double recovery_gain = 1.6;     // стартовое окно поиска = размер цели × коэффициент
    double recovery_grow = 1.15;    // рост окна поиска за каждый потерянный кадр

    // Подруливание
    int ap_deadband = 40;           // мёртвая зона ошибки, px
    int ap_max_offset = 80;         // макс. поправка канала
    int ap_pitch_forward = 100;     // подход вперёд (оффсет к нейтрали pitch)
    double ap_slew_rate = 15;       // макс. изменение поправки за кадр
    double ap_att_damp = 0.4;       // демпфирование по угловой скорости FC
    int ap_stick_deadband = 100;    // мёртвая зона стиков пилота (roll/pitch/yaw)
    int ap_throttle = 700;          // газ в режиме сопровождения (малина рулит — пульт не слушается)

    // Калибровка камеры (эго-компенсация)
    float cam_hfov = 70.0f;

    // Зуммер (GPIO). <0 — отключён
    int gpio_pin = -1;
    std::string gpio_chip = "0";
};

// ─── Инline-парсер tracker.ini ────────────────────────────────────────────
inline Config load_config(const std::string& path) {
    Config c;
    std::ifstream f(path);
    if (!f.is_open()) return c;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#' || line[0] == ';') continue;
        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string k = line.substr(0, eq), v = line.substr(eq + 1);
        // обрезать пробелы
        auto trim = [](std::string& s) {
            size_t a = s.find_first_not_of(" \t\r");
            size_t b = s.find_last_not_of(" \t\r");
            s = (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
        };
        trim(k); trim(v);
        if (k == "elrs_port") c.elrs_port = v;
        else if (k == "fc_port") c.fc_port = v;
        else if (k == "elrs_baud") c.elrs_baud = std::stoi(v);
        else if (k == "fc_baud") c.fc_baud = std::stoi(v);
        else if (k == "enable_crsf") c.enable_crsf = (v == "1" || v == "true" || v == "yes");
        else if (k == "camera_dev") c.camera_dev = std::stoi(v);
        else if (k == "camera_type") c.camera_type = v;
        else if (k == "cam_w") c.cam_w = std::stoi(v);
        else if (k == "cam_h") c.cam_h = std::stoi(v);
        else if (k == "cam_fps") c.cam_fps = std::stoi(v);
        else if (k == "disp_w") c.disp_w = std::stoi(v);
        else if (k == "disp_h") c.disp_h = std::stoi(v);
        else if (k == "fullscreen") c.fullscreen = (v == "1" || v == "true" || v == "yes");
        else if (k == "acquire_box") c.acquire_box = std::stoi(v);
        else if (k == "rc_neutral") c.rc_neutral = std::stoi(v);
        else if (k == "rc_engage_threshold") c.rc_engage_threshold = std::stoi(v);
        else if (k == "track_lost_frames") c.track_lost_frames = std::stoi(v);
        else if (k == "lost_timeout_s") c.lost_timeout_s = std::stod(v);
        else if (k == "recovery_gain") c.recovery_gain = std::stod(v);
        else if (k == "recovery_grow") c.recovery_grow = std::stod(v);
        else if (k == "ap_deadband") c.ap_deadband = std::stoi(v);
        else if (k == "ap_max_offset") c.ap_max_offset = std::stoi(v);
        else if (k == "ap_pitch_forward") c.ap_pitch_forward = std::stoi(v);
        else if (k == "ap_slew_rate") c.ap_slew_rate = std::stod(v);
        else if (k == "ap_att_damp") c.ap_att_damp = std::stod(v);
        else if (k == "ap_stick_deadband") c.ap_stick_deadband = std::stoi(v);
        else if (k == "ap_throttle") c.ap_throttle = std::stoi(v);
        else if (k == "cam_hfov") c.cam_hfov = std::stof(v);
        else if (k == "gpio_pin") c.gpio_pin = std::stoi(v);
        else if (k == "gpio_chip") c.gpio_chip = v;
    }
    return c;
}

// ─── Режимы слежения ──────────────────────────────────────────────────────
enum class TrackMode { MANUAL, LOCK, PREDICT, LOST };

// ─── Телеметрия FC (пишет crsf_bridge, читают egomotion/osd) ──────────────
struct Telemetry {
    bool armed = false;
    bool engaged = false;       // CH5 высокий
    float roll = 0, pitch = 0, yaw = 0;     // углы FC, град
    float alt = 0, climb = 0, speed = 0, heading = 0;
    float vcc = 0; int bat_pct = 0;
    std::vector<float> voltages;
    float lat = 0, lon = 0; int fix = 0, sats = 0;
    int ch5 = 992, ch6 = 992;
    std::array<int, 4> pilot{};     // CH1..CH4 от пилота (roll,pitch,gaz,yaw)
};

// ─── Состояние трекера (пишет tracking, читают osd/guidance/warn) ─────────
struct TrackState {
    TrackMode mode = TrackMode::MANUAL;
    bool active = false;
    cv::Rect roi;               // в координатах камеры (сырых)
    cv::Point center;           // центр цели, сырые координаты
    cv::Point2d vel;            // пиксельная скорость (для предсказания)
    float confidence = 0;
    int lost_frames = 0;
    double last_update = 0;
    double lost_since = 0;      // момент перехода в LOST
};

// ─── Общее состояние процесса ─────────────────────────────────────────────
struct Shared {
    std::mutex mtx_tlm, mtx_trk, mtx_ovr;
    Telemetry tlm;
    TrackState trk;

    std::atomic<bool> acquire_request{false};   // фронт CH5 → захват
    std::atomic<bool> needs_release{false};     // снят CH5 → сбросить трек

    // оффсеты подруливания (вычисляет guidance)
    bool have_offsets = false;
    int roll_off = 0, pitch_off = 0, yaw_off = 0;

    // баннер и зуммер
    std::atomic<bool> show_lost{false};

    // статистика
    std::atomic<uint64_t> frames{0}, overrides{0};
};

extern Shared g;   // определяется в main.cpp

} // namespace trk