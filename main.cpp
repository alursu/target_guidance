#include "common.h"
#include "crsf_bridge.h"
#include "egomotion.h"
#include "tracking.h"
#include "guidance.h"
#include "osd.h"
#include "warn.h"
#include "framesource.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include <atomic>
#include <cstdio>
#include <csignal>
#include <cstring>
#include <thread>
#include <chrono>
#include <cmath>

#include <unistd.h>

namespace trk {
Shared g;
}

using namespace trk;

static volatile sig_atomic_t g_quit = 0;
static void on_signal(int) { g_quit = 1; }
static bool cfg_no_crsf = false;

static cv::Mat g_sim_frame;         // последний сгенерированный sim-кадр
static std::atomic<int> g_sim_x{0}, g_sim_y{0};
static int g_sim_dx = 3, g_sim_dy = 2;

// генератор синтетических кадров: яркий движущийся квадрат на фоне
static void sim_loop() {
    cv::Mat f(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
    // лёгкий градиент и шум для работы CSRT/деталей
    cv::randn(f, cv::Scalar(60, 60, 60), cv::Scalar(8, 8, 8));
    cv::rectangle(f, cv::Rect(0, 0, 640, 40), cv::Scalar(90, 90, 90), -1);
    cv::rectangle(f, cv::Rect(0, 440, 640, 40), cv::Scalar(90, 90, 90), -1);
    int x = 260, y = 200;
    while (!g_quit) {
        {
            cv::Mat fr = f.clone();
            cv::rectangle(fr, cv::Rect(x, y, 48, 48), cv::Scalar(0, 200, 0), -1);
            cv::putText(fr, "TGT", cv::Point(x + 2, y + 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
            std::lock_guard<std::mutex> lk(g.mtx_trk);
            g_sim_frame = fr;
            g_sim_x.store(x); g_sim_y.store(y);
        }
        x += g_sim_dx; y += g_sim_dy;
        if (x < 8) g_sim_dx = 3;
        if (x + 48 > 632) g_sim_dx = -3;
        if (y < 8) g_sim_dy = 2;
        if (y + 48 > 432) g_sim_dy = -2;

        // имитация телеметрии: небольшое покачивание
        {
            std::lock_guard<std::mutex> lk(g.mtx_tlm);
            static double t = 0; t += 0.03;
            g.tlm.roll  = 12.0 * std::sin(t);
            g.tlm.pitch = 8.0 * std::sin(t * 1.3);
            g.tlm.yaw   = 5.0 * std::sin(t * 0.7);
            g.tlm.alt = 45 + 5 * std::sin(t * 0.2);
            g.tlm.vcc = 16.2;
            g.tlm.bat_pct = 87;
            g.tlm.sats = 14; g.tlm.fix = 3;
            g.tlm.heading = std::fmod(40 + 3 * t, 360);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / 25));
    }
}

int main(int argc, char** argv) {
    bool sim = false;
    std::string ini = "tracker.ini";
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--sim") == 0) sim = true;
        else if (strcmp(argv[i], "--no-crsf") == 0) cfg_no_crsf = true;
        else if (strncmp(argv[i], "--ini=", 6) == 0) ini = argv[i] + 6;
        else sim = (argv[i][0] == '-');
    }

    trk::Config cfg = trk::load_config(ini);
    if (cfg_no_crsf) cfg.enable_crsf = false;
    if (!sim)
        fprintf(stderr, "[MAIN] конфиг: %s (elrs=%s@%d fc=%s@%d cam=%dx%d@%d)\n",
                ini.c_str(), cfg.elrs_port.c_str(), cfg.elrs_baud,
                cfg.fc_port.c_str(), cfg.fc_baud,
                cfg.cam_w, cfg.cam_h, cfg.cam_fps);

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    // реле CRSF — только если включено; в sim-режиме телеметрия имитируется
    trk::CrsfBridge bridge(&cfg);
    std::thread sim_thr;
    if (sim) {
        sim_thr = std::thread(sim_loop);
        fprintf(stderr, "[MAIN] SIM-режим (без UART)\n");
    } else if (!cfg.enable_crsf) {
        fprintf(stderr, "[MAIN] CRSF выключен — только камера\n");
    } else if (!bridge.start()) {
        fprintf(stderr, "[MAIN] CRSF не запущен — работа без телеметрии\n");
    }

    trk::Tracking tracking(&cfg);
    trk::Guidance guidance(&cfg);
    trk::Osd osd(&cfg);
    trk::Warn warn(&cfg);
    warn.start();
    osd.open_window();

    std::unique_ptr<trk::FrameSource> src;
    if (!sim) {
        src = trk::FrameSource::make(cfg);
        fprintf(stderr, "[MAIN] источник кадра: %s\n",
                cfg.camera_type == "guide" ? "guide (SDK)" : "v4l2");
        if (!src->open()) {
            fprintf(stderr, "[MAIN] камера (%s) не открылась — переключаюсь в SIM\n",
                    cfg.camera_type.c_str());
            sim = true;
            sim_thr = std::thread(sim_loop);
        } else if (!src->isOpened()) {
            fprintf(stderr, "[MAIN] камера открыта, но поток мёртв — переключаюсь в SIM\n");
            sim = true;
            sim_thr = std::thread(sim_loop);
        }
    }

    cv::Mat frame, gray, disp;
    trk::Telemetry prev_tlm;
    bool have_prev_tlm = false;
    double t_prev = -1;

    while (!g_quit) {
        auto now = std::chrono::steady_clock::now().time_since_epoch().count() * 1e-9;

        if (sim) {
            std::lock_guard<std::mutex> lk(g.mtx_trk);
            frame = g_sim_frame;
        } else {
            if (!src->read(frame) || frame.empty()) {
                static uint64_t noframes = 0;
                if (noframes++ == 25)  // ~1 с без кадров
                    fprintf(stderr, "[MAIN] кадры от камеры не приходят\n");
                usleep(1000);
                continue;
            }
        }

        if (frame.empty()) continue;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // ── захват/снятие по сигналу пилота (из CRSF или имитации) ──
        bool acquire, release;
        {
            std::lock_guard<std::mutex> lk(g.mtx_trk);
            acquire = g.acquire_request.exchange(false);
            release = g.needs_release.exchange(false);
        }
        if (release) tracking.release();
        if (acquire && !sim) tracking.acquire(frame);

        // в sim-режиме: периодически имитируем сигнал захвата
        if (sim && !tracking.active() && (uint64_t)g.frames.load() % 60 == 40)
            tracking.acquire(frame);

        // ── эго-компенсация по углам FC ──
        trk::EgoShift ego;
        {
            std::lock_guard<std::mutex> lk(g.mtx_tlm);
            if (have_prev_tlm)
                ego = trk::ego_from_attitude(prev_tlm, g.tlm, cfg);
            prev_tlm = g.tlm;
            have_prev_tlm = true;
        }

        tracking.update(frame, gray, now, ego);

        // ── подруливание ──
        bool engaged;
        {
            std::lock_guard<std::mutex> lk(g.mtx_tlm);
            engaged = g.tlm.engaged;
        }
        if (engaged && tracking.active()) {
            guidance.update();   // вне rlock — внутри update() есть свои блокировки
        } else {
            std::lock_guard<std::mutex> lk2(g.mtx_ovr);
            g.have_offsets = false;
            guidance.reset();
        }

        // ── OSD ──
        if (cfg.fullscreen) cv::resize(frame, disp, cv::Size(cfg.disp_w, cfg.disp_h), 0, 0, cv::INTER_NEAREST);
        else disp = frame.clone();
        osd.draw(disp, frame, now, (uint32_t)g.frames.load());
        cv::imshow("OSD", disp);

        // ── таймаут LOST (лимит по времени) → возврат пилоту ──
        bool timed_out;
        {
            std::lock_guard<std::mutex> lk(g.mtx_trk);
            timed_out = (g.trk.mode == TrackMode::LOST &&
                         now - g.trk.lost_since > cfg.lost_timeout_s &&
                         g.trk.active);
        }
        if (timed_out) {                    // release() сам берёт mtx_trk
            fprintf(stderr, "[MAIN] LOST по таймауту → возврат пилоту\n");
            tracking.release();
        }

        g.frames.fetch_add(1);
        uint64_t fr = g.frames.load();
        if (fr % 50 == 0 && fr > 0 && t_prev > 0) {
            double dt = now - t_prev;
            fprintf(stderr, "[MAIN] кадры=%llu  %4.1f fps\n",
                    (unsigned long long)fr, 50.0 / dt);
            t_prev = now;
        }

        int key = cv::waitKey(1);
        if (key == 27 || key == 'q') break;
        if (key == 'e') tracking.acquire(frame);       // ручной захват
        if (key == 'r') tracking.release();

        if (t_prev < 0) t_prev = now;
    }

    g_quit = 1;
    bridge.stop();
    warn.stop();
    if (sim_thr.joinable()) sim_thr.join();
    cv::destroyAllWindows();
    return 0;
}