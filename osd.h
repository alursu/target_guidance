#pragma once

#include "common.h"
#include <opencv2/core.hpp>

namespace trk {

// ─── OSD-оверлей поверх видеокадра ──────────────────────────────────────
class Osd {
public:
    Osd(Config* cfg);

    // полноэкранное окно
    void open_window();
    void draw(cv::Mat& frame_out, const cv::Mat& raw, double now, uint32_t frame_no);

private:
    Config& cfg_;
    double last_print_ = 0;
};

} // namespace trk