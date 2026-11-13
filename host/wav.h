// Minimal WAV I/O for the host programs (host/render.cpp). Reads PCM 16/24/32-bit, IEEE float32 and
// WAVE_FORMAT_EXTENSIBLE with either subformat; returns channel 0 (the pedal's L In). Writes mono IEEE float32.
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace btwav {

namespace detail {
inline uint32_t U16(const uint8_t* p) { return uint32_t(p[0]) | uint32_t(p[1]) << 8; }
inline uint32_t U32(const uint8_t* p) { return U16(p) | U16(p + 2) << 16; }
inline void Put16(std::vector<uint8_t>& b, uint32_t v) {
    b.push_back(uint8_t(v));
    b.push_back(uint8_t(v >> 8));
}
inline void Put32(std::vector<uint8_t>& b, uint32_t v) {
    Put16(b, v & 0xFFFF);
    Put16(b, v >> 16);
}
inline void PutTag(std::vector<uint8_t>& b, const char* tag) { b.insert(b.end(), tag, tag + 4); }
}  // namespace detail

inline bool ReadWav(const char* path, std::vector<float>& ch0, uint32_t& rate, std::string& err) {
    using namespace detail;
    FILE* f = std::fopen(path, "rb");
    if (!f) {
        err = std::string("cannot open ") + path;
        return false;
    }
    std::vector<uint8_t> b;
    uint8_t chunk[65536];
    for (size_t n; (n = std::fread(chunk, 1, sizeof chunk, f)) > 0;) b.insert(b.end(), chunk, chunk + n);
    std::fclose(f);
    if (b.size() < 12 || std::memcmp(b.data(), "RIFF", 4) || std::memcmp(b.data() + 8, "WAVE", 4)) {
        err = std::string(path) + " is not a RIFF/WAVE file";
        return false;
    }
    uint32_t format = 0, channels = 0, bits = 0;
    bool haveFmt = false;
    for (size_t pos = 12; pos + 8 <= b.size();) {
        const uint8_t* c = b.data() + pos;
        const size_t size = U32(c + 4), body = pos + 8;
        const size_t avail = body + size <= b.size() ? size : b.size() - body;  // a truncated last chunk is read as is
        if (!std::memcmp(c, "fmt ", 4) && avail >= 16) {
            format = U16(c + 8);
            channels = U16(c + 10);
            rate = U32(c + 12);
            bits = U16(c + 22);
            if (format == 0xFFFE && avail >= 40) format = U16(c + 32);  // the subformat GUID's first two bytes
            haveFmt = true;
        } else if (!std::memcmp(c, "data", 4)) {
            if (!haveFmt) {
                err = std::string(path) + ": data chunk before fmt";
                return false;
            }
            const bool pcm = format == 1 && (bits == 16 || bits == 24 || bits == 32);
            const bool flt = format == 3 && bits == 32;
            if (!(pcm || flt) || channels == 0) {
                err = std::string(path) + ": unsupported format " + std::to_string(format) + ", " +
                      std::to_string(bits) + " bits";
                return false;
            }
            const size_t bytes = bits / 8, frame = bytes * channels, frames = avail / frame;
            ch0.resize(frames);
            for (size_t i = 0; i < frames; ++i) {
                const uint8_t* s = c + 8 + i * frame;
                if (flt) {
                    const uint32_t u = U32(s);
                    std::memcpy(&ch0[i], &u, 4);
                } else if (bits == 16) {
                    ch0[i] = float(int16_t(U16(s))) / 32768.0f;
                } else if (bits == 24) {
                    const uint32_t u = uint32_t(s[0]) << 8 | uint32_t(s[1]) << 16 | uint32_t(s[2]) << 24;
                    ch0[i] = float(int32_t(u) >> 8) / 8388608.0f;
                } else {
                    ch0[i] = float(double(int32_t(U32(s))) / 2147483648.0);
                }
            }
            return true;
        }
        pos = body + size + (size & 1);  // chunks are padded to even sizes
    }
    err = std::string(path) + ": no data chunk";
    return false;
}

inline bool WriteWavFloat(const char* path, const float* x, size_t n, uint32_t rate, std::string& err) {
    using namespace detail;
    std::vector<uint8_t> b;
    const uint32_t data = uint32_t(n * 4);
    PutTag(b, "RIFF");
    Put32(b, 4 + (8 + 18) + (8 + 4) + (8 + data));
    PutTag(b, "WAVE");
    PutTag(b, "fmt ");
    Put32(b, 18);
    Put16(b, 3);  // IEEE float
    Put16(b, 1);
    Put32(b, rate);
    Put32(b, rate * 4);
    Put16(b, 4);
    Put16(b, 32);
    Put16(b, 0);  // cbSize
    PutTag(b, "fact");
    Put32(b, 4);
    Put32(b, uint32_t(n));
    PutTag(b, "data");
    Put32(b, data);
    for (size_t i = 0; i < n; ++i) {
        uint32_t u;
        std::memcpy(&u, &x[i], 4);
        Put32(b, u);
    }
    FILE* f = std::fopen(path, "wb");
    if (!f) {
        err = std::string("cannot write ") + path;
        return false;
    }
    const bool ok = std::fwrite(b.data(), 1, b.size(), f) == b.size();
    if (std::fclose(f) != 0 || !ok) {
        err = std::string("error writing ") + path;
        return false;
    }
    return true;
}

}  // namespace btwav
