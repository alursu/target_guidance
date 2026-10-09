#include "crsf_bridge.h"

#include <array>
#include <cstring>
#include <thread>
#include <unistd.h>
#include <fcntl.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <cstdio>
#include <cerrno>
#include <chrono>
#include <atomic>

#ifndef TCGETS2
#define TCGETS2 0x802C542A
#endif
#ifndef TCSETS2
#define TCSETS2 0x402C542B
#endif
#ifndef BOTHER
#define BOTHER 0010000
#endif

namespace trk {

// ─── CRC-8/D5 (Полином 0xD5) ─────────────────────────────────────────────
uint8_t crc8_d5(uint8_t crc, uint8_t b) {
    crc ^= b;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0xD5);
        else crc <<= 1;
    }
    return crc;
}

static uint8_t crc8_frame(const uint8_t* data, size_t len) {
    uint8_t c = 0;
    for (size_t i = 0; i < len; i++) c = crc8_d5(c, data[i]);
    return c;
}

// ─── Открытие UART с нестандартной скоростью ────────────────────────────
bool open_crsf_port(int* fd_out, const std::string& path, int baud) {
    int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return false;

    struct termios2 t2;
    memset(&t2, 0, sizeof(t2));
    if (ioctl(fd, TCGETS2, &t2) != 0) {
        ::close(fd); return false;
    }
    t2.c_cflag &= ~(CBAUD | CBAUDEX);
    t2.c_cflag |= (BOTHER | CS8 | CLOCAL | CREAD);
    t2.c_ispeed = (speed_t)baud;
    t2.c_ospeed = (speed_t)baud;
    t2.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    t2.c_oflag &= ~OPOST;
    t2.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    t2.c_cc[VMIN] = 0;
    t2.c_cc[VTIME] = 0;
    if (ioctl(fd, TCSETS2, &t2) != 0) {
        ::close(fd); return false;
    }
    *fd_out = fd;
    return true;
}

int bytes_available(int fd) {
    int n = 0;
    if (ioctl(fd, FIONREAD, &n) != 0) return 0;
    return n;
}

// ─── 16 каналов × 11 бит ────────────────────────────────────────────────
std::array<int, 16> unpack_channels(const uint8_t* payload, size_t len) {
    std::array<int, 16> ch{};
    int bit_idx = 0;
    for (int c = 0; c < 16; c++) {
        int val = 0;
        for (int bit = 0; bit < 11; bit++) {
            int byte_pos = bit_idx / 8, bit_pos = bit_idx % 8;
            if (byte_pos < (int)len && (payload[byte_pos] & (1 << bit_pos))) {
                val |= (1 << bit);
            }
            bit_idx++;
        }
        ch[c] = val;
    }
    return ch;
}

std::vector<uint8_t> pack_channels(const std::array<int, 16>& ch) {
    std::vector<uint8_t> payload(22, 0);
    int bit_idx = 0;
    for (int c = 0; c < 16; c++) {
        for (int bit = 0; bit < 11; bit++) {
            if (ch[c] & (1 << bit)) {
                int byte_pos = bit_idx / 8, bit_pos = bit_idx % 8;
                payload[byte_pos] |= (1 << bit_pos);
            }
            bit_idx++;
        }
    }
    return payload;
}

// ─── Поиск целого CRSF-кадра в буфере ───────────────────────────────────
// Возвращает true и 2+len байт кадра, вырезает его из buf.
bool find_crsf_frame(std::vector<uint8_t>& buf, uint8_t* frame, size_t* frame_len) {
    while (!buf.empty() && buf[0] != 0xC8) buf.erase(buf.begin());
    if (buf.size() < 2) return false;
    size_t length = buf[1];
    if (length < 2 || length > 64) {
        buf.erase(buf.begin());
        return false;
    }
    if (buf.size() < 2 + length) return false;
    size_t total = 2 + length;
    memcpy(frame, buf.data(), total);
    *frame_len = total;
    buf.erase(buf.begin(), buf.begin() + total);
    return true;
}

// ─── Парсинг телеметрии ──────────────────────────────────────────────────
static void parse_crsf_telemetry(const uint8_t* frame, size_t len, Telemetry& tlm) {
    if (len < 4) return;
    uint8_t ftype = frame[2];
    const uint8_t* p = frame + 3;
    size_t plen = len - 4; // без типа и CRC

auto s16 = [&](size_t i) -> int {
        if (i + 1 >= plen) return 0;
        int v = (p[i] << 8) | p[i + 1];      // CRSF: big-endian
        if (v & 0x8000) v -= 0x10000;
        return v;
    };

    if (ftype == 0x08 && plen >= 8) {           // BATTERY
        int v = (p[0] << 8) | p[1];
        tlm.vcc = v * 0.1f;
        tlm.bat_pct = p[7];
        tlm.voltages.clear();
        tlm.voltages.push_back(v * 0.1f);
    } else if (ftype == 0x09 && plen >= 3) {    // BARO_ALT
        int alt_dec = (p[0] << 8) | p[1];
        tlm.alt = (alt_dec - 10000) / 10.0f;
        int vs = p[2];
        if (vs & 0x80) vs -= 0x100;
        tlm.climb = vs * 0.1f;
    } else if (ftype == 0x02 && plen >= 15) {   // GPS
        int32_t lat = (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3];
        int32_t lon = (p[4] << 24) | (p[5] << 16) | (p[6] << 8) | p[7];
        if (lat & 0x80000000) lat -= 0x100000000;
        if (lon & 0x80000000) lon -= 0x100000000;
        tlm.lat = lat / 1e7f;
        tlm.lon = lon / 1e7f;
        int alt_g = (p[8] << 8) | p[9];
        if (alt_g & 0x8000) alt_g -= 0x10000;
        tlm.alt = alt_g * 0.1f;
        tlm.speed = ((p[10] << 8) | p[11]) * 0.01f;
        tlm.heading = ((p[12] << 8) | p[13]) * 0.1f;
        tlm.sats = p[14];
        if (plen >= 16) tlm.fix = p[15];
    } else if (ftype == 0x1E && plen >= 6) {    // ATTITUDE
        int pitch = s16(0), roll = s16(2), yaw = s16(4);
        tlm.pitch = pitch * 0.00572958f;
        tlm.roll  = roll * 0.00572958f;
        tlm.yaw   = yaw * 0.00572958f;
    } else if (ftype == 0x29 && plen >= 4) {    // ALT_MSL
        int32_t a32 = (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3];
        tlm.alt = a32 * 0.01f;
    }
}

// ─── Подмена каналов (вызывается при CH5 высоком) ────────────────────────
// «Малина — мост»: пока трек выключен — чистое пропускание пульт→FC,
// при включённом треке — FC получает команды ОТ МАЛИНЫ, пульт не слушается.
// Наследует логику combined_crsf_osd.py (process_rc_frame + fuzzy).
void CrsfBridge::apply_channel_overrides(std::array<int, 16>& ch) {
    int neutral = cfg_.rc_neutral;
    // база: нейтраль по всем осям + фиксированный газ (малина рулит сама)
    ch[0] = neutral;                       // roll
    ch[1] = neutral + cfg_.ap_pitch_forward; // pitch вперёд при подходе
    ch[2] = cfg_.ap_throttle;              // газ
    ch[3] = neutral;                       // yaw
    {
        std::lock_guard<std::mutex> lk(g.mtx_ovr);
        if (g.have_offsets) {
            ch[0] = neutral + g.roll_off;
            ch[1] = neutral + g.pitch_off + cfg_.ap_pitch_forward;
            ch[3] = neutral + g.yaw_off;
        }
    }
    g.overrides.fetch_add(1);
}

// ─── Потоки реле ──────────────────────────────────────────────────────────
void CrsfBridge::relay_elrs_to_fc() {
    std::vector<uint8_t> buf;
    uint8_t frame[128];
    size_t flen = 0;
    uint8_t rbuf[4096];
    unsigned rc_count = 0, tlm_count = 0, other = 0, fwd_count = 0, wr_err = 0;
    auto t0 = std::chrono::steady_clock::now();
    bool rx_was_ok = false;
    while (running_) {
        int n = bytes_available(fd_elrs_);
        if (n <= 0) { usleep(500); continue; }
        if (n > (int)sizeof(rbuf)) n = sizeof(rbuf);
        int r = ::read(fd_elrs_, rbuf, n);
        if (r <= 0) { usleep(500); continue; }
        buf.insert(buf.end(), rbuf, rbuf + r);
        while (find_crsf_frame(buf, frame, &flen)) {
            if (flen < 5) continue;
            uint8_t ftype = frame[2];
            if (ftype == 0x16) rc_count++;            // RC channels
            else if (ftype == 0x14 || ftype == 0x1A || ftype == 0x2B) tlm_count++; // telem. от ELRS
            else other++;
            if (ftype == 0x16) {
                const uint8_t* payload = frame + 3;
                size_t plen = flen - 4;
                if (plen >= 22) {
                    auto ch = unpack_channels(payload, plen);
                    int ch5 = ch[4], ch6 = ch[5];
                    {
                        std::lock_guard<std::mutex> lk(g.mtx_tlm);
                        g.tlm.ch5 = ch5;
                        g.tlm.ch6 = ch6;
                        g.tlm.pilot = {ch[0], ch[1], ch[2], ch[3]};
                        bool engage = ch5 > cfg_.rc_engage_threshold;
                        if (engage && !g.tlm.engaged) {
                            // фронт сигнала → захват квадрата в центре
                            g.acquire_request = true;
                        }
                        if (!engage && g.tlm.engaged) {
                            g.needs_release = true;
                        }
                        g.tlm.engaged = engage;
                    }
                    // в режиме сопровождения FC получает команды от малины
                    if (g.tlm.engaged) {
                        apply_channel_overrides(ch);
                        auto pl = pack_channels(ch);
                        frame[1] = (uint8_t)(pl.size() + 2);
                        memcpy(frame + 3, pl.data(), pl.size());
                        frame[3 + pl.size()] = crc8_frame(frame + 2, 1 + pl.size());
                        flen = 3 + pl.size() + 1;
                    }
                }
            }
            ssize_t w = ::write(fd_fc_, frame, flen);
            if (w < 0) {
                char e[96];
                snprintf(e, sizeof e,
                         "[CRSF] FC: ОШИБКА записи fd=%d errno=%d (%s)",
                         fd_fc_, errno, strerror(errno));
                if (wr_err++ % 20 == 0) fprintf(stderr, "%s\n", e);
            } else fwd_count++;
        }
        auto now_t = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now_t - t0).count() >= 2) {
            if (rc_count == 0) {
                if (dead_ < 2)   // 4 первые секунды молчания важнее всего
                    fprintf(stderr, "[CRSF] ELRS: RC-кадров НЕТ — проверь порт/бод/проводку\n");
                dead_++;
            } else {
                fprintf(stderr, "[CRSF] ELRS OK: %u RC + %u tlm/2с → FC: %u ведено%s%s\n",
                        rc_count, (unsigned)(tlm_count + other), fwd_count,
                        wr_err ? " ОШИБКИ ЗАПИСИ!" : "",
                        dead_ ? " (снова появились)" : "");
                dead_ = 0;
            }
            rc_count = tlm_count = other = fwd_count = wr_err = 0;
            t0 = now_t;
        }
    }
}

void CrsfBridge::relay_fc_to_elrs() {
    std::vector<uint8_t> buf;
    uint8_t frame[128];
    size_t flen = 0;
    uint8_t rbuf[4096];
    unsigned fc_rx = 0;
    auto t0 = std::chrono::steady_clock::now();
    while (running_) {
        int n = bytes_available(fd_fc_);
        if (n <= 0) { usleep(500); continue; }
        if (n > (int)sizeof(rbuf)) n = sizeof(rbuf);
        int r = ::read(fd_fc_, rbuf, n);
        if (r <= 0) { usleep(500); continue; }
        buf.insert(buf.end(), rbuf, rbuf + r);
        while (find_crsf_frame(buf, frame, &flen)) {
            if (flen < 5) continue;
            fc_rx++;
            {
                std::lock_guard<std::mutex> lk(g.mtx_tlm);
                parse_crsf_telemetry(frame, flen, g.tlm);
            }
            ::write(fd_elrs_, frame, flen);
        }
        auto now_t = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now_t - t0).count() >= 2) {
            fprintf(stderr, "[CRSF] FC: %u кадр(ов) телеметрии/2с — FC жив и отвечает\n", fc_rx);
            fc_rx = 0;
            t0 = now_t;
        }
    }
}

// ─── Управление ───────────────────────────────────────────────────────────
bool CrsfBridge::start() {
    if (!open_crsf_port(&fd_elrs_, cfg_.elrs_port, cfg_.elrs_baud)) {
        fprintf(stderr, "[CRSF] не удалось открыть %s@%d\n",
                cfg_.elrs_port.c_str(), cfg_.elrs_baud);
        return false;
    }
    fprintf(stderr, "[CRSF] открыт ELRS: %s@%d\n",
            cfg_.elrs_port.c_str(), cfg_.elrs_baud);
    if (!open_crsf_port(&fd_fc_, cfg_.fc_port, cfg_.fc_baud)) {
        fprintf(stderr, "[CRSF] не удалось открыть %s@%d\n",
                cfg_.fc_port.c_str(), cfg_.fc_baud);
        return false;
    }
    fprintf(stderr, "[CRSF] открыт FC: %s@%d\n",
            cfg_.fc_port.c_str(), cfg_.fc_baud);
    fprintf(stderr, "[CRSF] реле: %s@%d ⇄ %s@%d (пульт→FC; в треке FC от малины)\n",
            cfg_.elrs_port.c_str(), cfg_.elrs_baud,
            cfg_.fc_port.c_str(), cfg_.fc_baud);
    running_ = true;
    std::thread(&CrsfBridge::relay_elrs_to_fc, this).detach();
    std::thread(&CrsfBridge::relay_fc_to_elrs, this).detach();
    return true;
}

void CrsfBridge::stop() {
    running_ = false;
    if (fd_elrs_ >= 0) ::close(fd_elrs_);
    if (fd_fc_ >= 0) ::close(fd_fc_);
}

} // namespace trk