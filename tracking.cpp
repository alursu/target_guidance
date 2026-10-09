#include "tracking.h"

#include <opencv2/tracking.hpp>
#include <opencv2/video/tracking.hpp>
#include <cmath>
#include <algorithm>
#include <cstdio>

namespace trk {

cv::Rect expanded_window(const cv::Rect& roi, const cv::Point& center, double gain, int lost,
                         const cv::Size& frame, double grow) {
    double w = std::max(20, roi.width) * gain * std::pow(grow, std::max(0, lost - 1));
    double h = std::max(20, roi.height) * gain * std::pow(grow, std::max(0, lost - 1));
    int wx = (int)std::max(24.0, w);
    int wy = (int)std::max(24.0, h);
    cv::Rect win(center.x - wx / 2, center.y - wy / 2, wx, wy);
    win &= cv::Rect(4, 4, frame.width - 8, frame.height - 8);
    return win;
}

void Tracking::acquire(const cv::Mat& frame) {
    int s = cfg_.acquire_box;
    int w = frame.cols, h = frame.rows;
    cv::Rect box(std::max(0, (w - s) / 2), std::max(0, (h - s) / 2), s, s);
    // квадрат в пределах кадра
    box &= cv::Rect(0, 0, w, h);
    if (box.width < 12 || box.height < 12) return;

    try {
        tracker_ = cv::TrackerCSRT::create();
    } catch (...) {
        tracker_ = cv::TrackerCSRT::create();
    }
    tracker_->init(frame, box);

    active_ = true;
    have_prev_ = false;

    {
        std::lock_guard<std::mutex> lk(g.mtx_trk);
        g.trk.mode = TrackMode::LOCK;
        g.trk.active = true;
        g.trk.roi = box;
        g.trk.center = cv::Point(box.x + box.width / 2, box.y + box.height / 2);
        g.trk.lost_frames = 0;
        g.trk.confidence = 1.0f;
        g.trk.last_update = prev_time_;
    }
    fprintf(stderr, "[TRACK] захват: %dx%d+%d,%d\n", box.width, box.height, box.x, box.y);
}

void Tracking::release() {
    active_ = false;
    tracker_.release();
    have_prev_ = false;
    {
        std::lock_guard<std::mutex> lk(g.mtx_trk);
        g.trk.mode = TrackMode::MANUAL;
        g.trk.active = false;
        g.trk.roi = cv::Rect();
        g.trk.confidence = 0;
        g.trk.lost_frames = 0;
    }
    fprintf(stderr, "[TRACK] снятие — MANUAL\n");
}

void Tracking::update(const cv::Mat& frame, const cv::Mat& gray, double now, const EgoShift& ego) {
    if (!active_ || !tracker_) return;

    cv::Rect roi;
    bool ok = tracker_->update(frame, roi);
    if (roi.width < 8 || roi.height < 8) ok = false;

    {
        std::lock_guard<std::mutex> lk(g.mtx_trk);
        g.trk.last_update = now;

        if (ok) {
            cv::Point c(roi.x + roi.width / 2, roi.y + roi.height / 2);
            if (have_prev_ && now - prev_time_ > 1e-6) {
                double dt = now - prev_time_;
                g.trk.vel = g.trk.vel * 0.7 +
                            cv::Point2d(c.x - prev_center_.x, c.y - prev_center_.y) / dt * 0.3;
            }
            prev_center_ = c;
            prev_time_ = now;
            have_prev_ = true;

            g.trk.roi = roi;
            g.trk.center = c;
            g.trk.mode = TrackMode::LOCK;
            g.trk.lost_frames = 0;
            g.trk.confidence = 1.0f;
        } else {
            g.trk.lost_frames++;
            double dt = now - prev_time_;
            if (g.trk.lost_frames < cfg_.track_lost_frames) {
                // предсказание по накопленной скорости
                cv::Point2d next = cv::Point2d(g.trk.center) + g.trk.vel * dt;
                g.trk.center = cv::Point((int)next.x, (int)next.y);
                if (g.trk.mode != TrackMode::PREDICT) {
                    g.trk.lost_since = now;
                    g.trk.mode = TrackMode::PREDICT;
                }
            } else {
                if (g.trk.mode != TrackMode::LOST) g.trk.lost_since = now;
                g.trk.mode = TrackMode::LOST;
            }
        }
    }

    // восстановление при потере (по остаточному движению)
    if (g.trk.mode == TrackMode::LOST && !prev_gray_.empty()) {
        cv::Rect win;
        {
            std::lock_guard<std::mutex> lk(g.mtx_trk);
            cv::Rect roi0 = g.trk.roi;
            cv::Point c = g.trk.center;
            int lost = g.trk.lost_frames;
            win = expanded_window(roi0, c, cfg_.recovery_gain, lost,
                                  gray.size(), cfg_.recovery_grow);
        }
        cv::Rect rec = findTargetByMotionAnomaly(prev_gray_, gray, win, ego);
        if (rec.area() > 400) {
            try { tracker_ = cv::TrackerCSRT::create(); }
            catch (...) { tracker_ = cv::TrackerCSRT::create(); }
            tracker_->init(frame, rec);
            fprintf(stderr, "[TRACK] восстановление по движению: %dx%d+%d,%d\n",
                    rec.width, rec.height, rec.x, rec.y);
            {
                std::lock_guard<std::mutex> lk(g.mtx_trk);
                g.trk.roi = rec;
                g.trk.center = cv::Point(rec.x + rec.width / 2, rec.y + rec.height / 2);
                g.trk.lost_frames = 0;
                g.trk.mode = TrackMode::LOCK;
                g.trk.confidence = 0.9f;
                g.trk.last_update = now;
            }
            prev_center_ = g.trk.center;
            prev_time_ = now;
            have_prev_ = true;
        }
    }

    if (!gray.empty()) prev_gray_ = gray.clone();
}

// ─── Восстановление по аномалии остаточного движения (из main.cpp) ──────
// Порт с двумя улучшениями:
//  1) поиск ограничен окном вокруг предсказанной позиции (window-маска),
//  2) базовый вектор фона = компенсированное собственным движением камеры
//     значение (если углы FC свежие), иначе — медиана потока (старый способ).
cv::Rect Tracking::findTargetByMotionAnomaly(const cv::Mat& prev_gray,
                                             const cv::Mat& curr_gray,
                                             const cv::Rect& window,
                                             const EgoShift& ego) {
    // маска поиска: разрешаем точки только внутри окна (чуть сдвинуты от краёв)
    cv::Mat feature_mask = cv::Mat::zeros(prev_gray.size(), CV_8U);
    cv::Rect w = window & cv::Rect(0, 0, prev_gray.cols, prev_gray.rows);
    if (w.width < 12 || w.height < 12) return cv::Rect();
    feature_mask(w) = 255;

    std::vector<cv::Point2f> prev_pts;
    cv::goodFeaturesToTrack(prev_gray, prev_pts, 300, 0.01, 10, feature_mask);
    if (prev_pts.size() < 8) return cv::Rect();

    std::vector<cv::Point2f> curr_pts;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_gray, curr_gray, prev_pts, curr_pts,
                             status, err, cv::Size(21, 21), 3);

    std::vector<cv::Point2f> valid_prev, valid_curr, motion_vectors;
    for (size_t i = 0; i < status.size(); i++) {
        if (!status[i]) continue;
        valid_prev.push_back(prev_pts[i]);
        valid_curr.push_back(curr_pts[i]);
        motion_vectors.push_back(curr_pts[i] - prev_pts[i]);
    }
    if (motion_vectors.size() < 8) return cv::Rect();

    // базовый вектор фона
    cv::Point2f base;
    if (ego.valid) {
        base = cv::Point2f((float)ego.dx, (float)ego.dy);
    } else {
        std::vector<float> dx, dy;
        for (auto& v : motion_vectors) { dx.push_back(v.x); dy.push_back(v.y); }
        std::sort(dx.begin(), dx.end());
        std::sort(dy.begin(), dy.end());
        base = cv::Point2f(dx[dx.size() / 2], dy[dy.size() / 2]);
    }

    std::vector<float> residuals;
    std::vector<cv::Point2f> res_pts;
    for (size_t i = 0; i < motion_vectors.size(); i++) {
        float rx = motion_vectors[i].x - base.x;
        float ry = motion_vectors[i].y - base.y;
        float r = std::sqrt(rx * rx + ry * ry);
        residuals.push_back(r);
        res_pts.push_back(cv::Point2f(rx, ry));
    }

    std::vector<float> sorted = residuals;
    std::sort(sorted.begin(), sorted.end());
    float med = sorted[sorted.size() / 2];
    float threshold = std::max(med * 2.5f, 3.0f);

    std::vector<cv::Point2f> anomaly_pts;
    for (size_t i = 0; i < residuals.size(); i++) {
        if (residuals[i] > threshold) {
            anomaly_pts.push_back(valid_curr[i]);
        }
    }
    if (anomaly_pts.size() < 3) return cv::Rect();

    cv::Rect bbox = cv::boundingRect(anomaly_pts);
    bbox &= window;
    if (bbox.area() < 50) return cv::Rect();

    int pad_x = (int)(bbox.width * 0.3f), pad_y = (int)(bbox.height * 0.3f);
    cv::Rect padded(bbox.x - pad_x, bbox.y - pad_y,
                    bbox.width + pad_x * 2, bbox.height + pad_y * 2);
    padded &= cv::Rect(0, 0, prev_gray.cols, prev_gray.rows);
    return padded;
}

} // namespace trk