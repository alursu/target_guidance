#include "framesource.h"

#ifdef TRK_GUIDE_CAMERA
#include "guidescusb2.h"
#endif
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <chrono>
#include <condition_variable>
#include <unistd.h>

namespace trk {

// ─── V4L2: обычная камера через OpenCV VideoCapture ──────────────────────
class V4l2FrameSource : public FrameSource {
public:
    explicit V4l2FrameSource(const Config& cfg) : cfg_(cfg) {}

    bool open() override {
        if (cfg_.camera_type == "guide") {
            fprintf(stderr, "[CAM] в конфиге camera_type=guide, но бинарь собран без "
                    "Guide SDK (нет camera_sdk рядом с проектом)\n");
            return false;
        }
        cap_.open(cfg_.camera_dev);
        if (!cap_.isOpened()) return false;
        cap_.set(cv::CAP_PROP_FRAME_WIDTH,  cfg_.cam_w);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cfg_.cam_h);
        cap_.set(cv::CAP_PROP_FPS,          cfg_.cam_fps);
        cap_.set(cv::CAP_PROP_BUFFERSIZE,   1);
        return true;
    }

    bool read(cv::Mat& out) override {
        return cap_.read(out);
    }

    bool isOpened() const override { return cap_.isOpened(); }
    void close() override { cap_.release(); }

private:
    const Config& cfg_;
    cv::VideoCapture cap_;
};

#ifdef TRK_GUIDE_CAMERA

// ─── Guide: тепловизионная USB-камера (микро-SDK libGuideSCUSB2) ─────────
// Кадры 640×512 UYVY приходят в колбэк из потока SDK; здесь храним новейший
// кадр и отдаём его в read() (блокируемся до появления нового).
class GuideFrameSource : public FrameSource {
public:
    GuideFrameSource() = default;
    ~GuideFrameSource() override { close(); }

    bool open() override;
    bool read(cv::Mat& out) override;
    bool isOpened() const override { return inited_; }
    void close() override;

    // вызывается из колбэка SDK
    void push(cv::Mat&& f);

private:
    bool inited_ = false;
    std::mutex m_;
    std::condition_variable cv_;
    cv::Mat latest_;
    uint64_t gen_ = 0;
};

static GuideFrameSource* g_guide_self = nullptr;

#define GUIDE_ID 1

int guideSerialCb(int id, guide_usb_serial_data_t* p);
int guideStatusCb(int id, guide_usb_device_status_e st);
int guideFrameCb(int id, guide_usb_frame_data_t* p);

bool GuideFrameSource::open() {
    guide_usb_setloglevel(LOG_LEVEL_ERROR);

    int count = guide_usb_get_devcount();
    int tries = 0;
    while (count < 1 && tries < 300) {   // до ~5 минут ожидания камеры
        usleep(1000000);
        count = guide_usb_get_devcount();
        tries++;
    }
    if (count < 1) {
        fprintf(stderr, "[GUIDE] камера не найдена\n");
        return false;
    }
    fprintf(stderr, "[GUIDE] устройств: %d\n", count);

    if (guide_usb_initial(GUIDE_ID) < 0) {
        fprintf(stderr, "[GUIDE] init fail\n");
        return false;
    }
    guide_usb_opencommandcontrol(GUIDE_ID, (OnSerialDataReceivedCB)guideSerialCb);

    unsigned char adaptive[] = {0x55,0xAA,0x07,0x01,0x00,0x07,0x00,0x00,0x00,0x01,0x00,0xF0};
    unsigned char save[]     = {0x55,0xAA,0x07,0x01,0x00,0x04,0x00,0x00,0x00,0x01,0x03,0xF0};
    guide_usb_sendcommand(GUIDE_ID, adaptive, sizeof(adaptive));
    guide_usb_sendcommand(GUIDE_ID, save, sizeof(save));

    g_guide_self = this;
    int ret = guide_usb_openstream_auto(GUIDE_ID, (OnFrameDataReceivedCB)guideFrameCb,
                                        (OnDeviceConnectStatusCB)guideStatusCb);
    if (ret < 0) {
        fprintf(stderr, "[GUIDE] openstream fail: %d\n", ret);
        g_guide_self = nullptr;
        return false;
    }
    inited_ = true;
    fprintf(stderr, "[GUIDE] поток открыт\n");
    return true;
}

void GuideFrameSource::push(cv::Mat&& f) {
    {
        std::lock_guard<std::mutex> lk(m_);
        latest_ = std::move(f);
        gen_++;
    }
    cv_.notify_one();
}

bool GuideFrameSource::read(cv::Mat& out) {
    std::unique_lock<std::mutex> lk(m_);
    uint64_t last = gen_;
    if (!cv_.wait_for(lk, std::chrono::milliseconds(500), [&]{ return gen_ > last; })) {
        if (latest_.empty()) return false;   // кадров ещё нет вообще
    }
    out = latest_.clone();
    return true;
}

void GuideFrameSource::close() {
    if (inited_) {
        guide_usb_closestream(GUIDE_ID);
        guide_usb_closecommandcontrol(GUIDE_ID);
        guide_usb_exit(GUIDE_ID);
        inited_ = false;
    }
    if (g_guide_self == this) g_guide_self = nullptr;
}

int guideSerialCb(int id, guide_usb_serial_data_t* p) { (void)id; (void)p; return 0; }
int guideStatusCb(int id, guide_usb_device_status_e st) {
    fprintf(stderr, "[GUIDE] %d: %s\n", id,
            (st == DEVICE_CONNECT_OK) ? "connected" : "disconnected");
    return 0;
}
int guideFrameCb(int id, guide_usb_frame_data_t* p) {
    if (id != GUIDE_ID || !p || !p->frame_yuv_data) return 0;
    if (!g_guide_self) return 0;
    cv::Mat yuv(512, 640, CV_8UC2, p->frame_yuv_data);
    cv::Mat bgr;
    cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_UYVY);
    g_guide_self->push(std::move(bgr));
    return 0;
}

#endif // TRK_GUIDE_CAMERA

// ─── Фабрика ─────────────────────────────────────────────────────────────
std::unique_ptr<FrameSource> FrameSource::make(const Config& cfg) {
#ifdef TRK_GUIDE_CAMERA
    if (cfg.camera_type == "guide")
        return std::make_unique<GuideFrameSource>();
#endif
    return std::make_unique<V4l2FrameSource>(cfg);
}

} // namespace trk