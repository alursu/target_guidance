#pragma once

#include "common.h"
#include <opencv2/core.hpp>
#include <memory>

namespace trk {

// ─── Единый источник видеокадров ─────────────────────────────────────────
// Две реализации: V4L2 (обычная камера через VideoCapture) и Guide
// (тепловизионная USB-камера со своим SDK). Выбор — в конфиге camera_type.
class FrameSource {
public:
    virtual ~FrameSource() = default;

    virtual bool open() = 0;          // открыть поток (может ждать камеру)
    virtual bool read(cv::Mat& out) = 0;  // блокирующий доставка кадра
    virtual bool isOpened() const = 0;
    virtual void close() = 0;

    // фабрика по cfg.camera_type
    static std::unique_ptr<FrameSource> make(const Config& cfg);
};

} // namespace trk