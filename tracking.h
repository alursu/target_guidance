#pragma once

#include "common.h"
#include "egomotion.h"
#include <opencv2/video/tracking.hpp>

namespace trk {

// ─── Слежение: захват по CH5 (квадрат А) + CSRT + восстановление ─────────
class Tracking {
public:
    Tracking(Config* cfg) : cfg_(*cfg) {}

    // Захват объекта из квадрата в центре кадра (квадрат А).
    void acquire(const cv::Mat& frame);

    // Кадровый шаг: обновление трекера, потеря, восстановление.
    // gray — 8-битный Cv; now — время в секундах (steady).
    void update(const cv::Mat& frame, const cv::Mat& gray, double now, const EgoShift& ego);

    // Снятие: CH5 отжат → режим MANUAL, трекер выгружается.
    void release();

    bool active() const { return active_; }

private:
    Config& cfg_;
    bool active_ = false;
    cv::Ptr<cv::Tracker> tracker_;
    cv::Mat prev_gray_;
    cv::Point prev_center_;
    double prev_time_ = 0;
    bool have_prev_ = false;

    // восстановление по аномалии остаточного движения (порт из main.cpp)
    cv::Rect findTargetByMotionAnomaly(const cv::Mat& prev_gray,
                                       const cv::Mat& curr_gray,
                                       const cv::Rect& window,
                                       const EgoShift& ego);
};

// Вспомогательное: прямоугольник поиска вокруг предсказанной позиции.
cv::Rect expanded_window(const cv::Rect& roi, const cv::Point& center, double gain, int lost,
                         const cv::Size& frame, double grow);

} // namespace trk