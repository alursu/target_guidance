#pragma once

#include "common.h"
#include <cstdint>
#include <vector>

namespace trk {

// ─── CRSF-реле: ELRS ⇄ FC, подмена каналов, телеметрия ──────────────────
class CrsfBridge {
public:
    CrsfBridge(Config* cfg) : cfg_(*cfg) {}

    // Открывает порты, поднимает потоки реле. Возвращает false при неудаче.
    bool start();
    void stop();

private:
    Config& cfg_;
    int fd_elrs_ = -1;
    int fd_fc_ = -1;
    bool running_ = false;
    int dead_ = 0;              // счётчик периодов молчания ELRS (диагностика)

    void relay_elrs_to_fc();
    void relay_fc_to_elrs();

    // наложение поправок подруливания на каналы (вызывается из relay_elrs_to_fc)
    void apply_channel_overrides(std::array<int, 16>& ch);
};

// ─── Низкоуровневые CRC/упаковка (используются тестами и реле) ────────────
uint8_t crc8_d5(uint8_t crc, uint8_t b);
std::array<int, 16> unpack_channels(const uint8_t* payload, size_t len);
std::vector<uint8_t> pack_channels(const std::array<int, 16>& ch);
bool find_crsf_frame(std::vector<uint8_t>& buf, uint8_t* frame, size_t* frame_len);
int  bytes_available(int fd);
bool open_crsf_port(int* fd, const std::string& path, int baud);

} // namespace trk