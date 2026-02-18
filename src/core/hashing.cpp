// About: SHA-256 implementation for incremental processing — self-contained,
// no OpenSSL dependency. Based on the FIPS 180-4 specification.

#include "simforge/core/hashing.h"

#include <array>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace simforge {

namespace {

// SHA-256 round constants
constexpr std::array<uint32_t, 64> K = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
    0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
    0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
    0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

inline uint32_t rotr(uint32_t x, uint32_t n) {
    return (x >> n) | (x << (32 - n));
}

inline uint32_t ch(uint32_t x, uint32_t y, uint32_t z) {
    return (x & y) ^ (~x & z);
}

inline uint32_t maj(uint32_t x, uint32_t y, uint32_t z) {
    return (x & y) ^ (x & z) ^ (y & z);
}

inline uint32_t sigma0(uint32_t x) {
    return rotr(x, 2) ^ rotr(x, 13) ^ rotr(x, 22);
}

inline uint32_t sigma1(uint32_t x) {
    return rotr(x, 6) ^ rotr(x, 11) ^ rotr(x, 25);
}

inline uint32_t gamma0(uint32_t x) {
    return rotr(x, 7) ^ rotr(x, 18) ^ (x >> 3);
}

inline uint32_t gamma1(uint32_t x) {
    return rotr(x, 17) ^ rotr(x, 19) ^ (x >> 10);
}

struct SHA256State {
    std::array<uint32_t, 8> h = {
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
        0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
    };
    uint64_t total_len = 0;
    std::array<uint8_t, 64> buffer{};
    size_t buffer_len = 0;

    void process_block(const uint8_t* block) {
        std::array<uint32_t, 64> w{};

        // Prepare message schedule
        for (int i = 0; i < 16; ++i) {
            w[i] = (static_cast<uint32_t>(block[i * 4]) << 24)
                 | (static_cast<uint32_t>(block[i * 4 + 1]) << 16)
                 | (static_cast<uint32_t>(block[i * 4 + 2]) << 8)
                 | (static_cast<uint32_t>(block[i * 4 + 3]));
        }
        for (int i = 16; i < 64; ++i) {
            w[i] = gamma1(w[i - 2]) + w[i - 7] + gamma0(w[i - 15]) + w[i - 16];
        }

        auto [a, b, c, d, e, f, g, hh] = h;

        for (int i = 0; i < 64; ++i) {
            uint32_t t1 = hh + sigma1(e) + ch(e, f, g) + K[i] + w[i];
            uint32_t t2 = sigma0(a) + maj(a, b, c);
            hh = g;
            g = f;
            f = e;
            e = d + t1;
            d = c;
            c = b;
            b = a;
            a = t1 + t2;
        }

        h[0] += a; h[1] += b; h[2] += c; h[3] += d;
        h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
    }

    void update(const uint8_t* data, size_t len) {
        total_len += len;

        // Fill buffer first
        if (buffer_len > 0) {
            size_t space = 64 - buffer_len;
            size_t copy = std::min(len, space);
            std::memcpy(buffer.data() + buffer_len, data, copy);
            buffer_len += copy;
            data += copy;
            len -= copy;

            if (buffer_len == 64) {
                process_block(buffer.data());
                buffer_len = 0;
            }
        }

        // Process full blocks
        while (len >= 64) {
            process_block(data);
            data += 64;
            len -= 64;
        }

        // Buffer remainder
        if (len > 0) {
            std::memcpy(buffer.data(), data, len);
            buffer_len = len;
        }
    }

    std::string finalize() {
        // Padding
        uint64_t bit_len = total_len * 8;
        uint8_t pad = 0x80;
        update(&pad, 1);

        // Pad with zeros until 56 bytes mod 64
        uint8_t zero = 0;
        while (buffer_len != 56) {
            update(&zero, 1);
        }

        // Append length in big-endian
        uint8_t len_bytes[8];
        for (int i = 7; i >= 0; --i) {
            len_bytes[i] = static_cast<uint8_t>(bit_len & 0xff);
            bit_len >>= 8;
        }
        update(len_bytes, 8);

        // Format as hex string
        std::ostringstream oss;
        for (auto word : h) {
            oss << std::hex << std::setfill('0') << std::setw(8) << word;
        }
        return oss.str();
    }
};

}  // namespace

std::string sha256_bytes(const uint8_t* data, size_t len) {
    SHA256State state;
    state.update(data, len);
    return state.finalize();
}

std::string sha256_string(const std::string& input) {
    return sha256_bytes(reinterpret_cast<const uint8_t*>(input.data()), input.size());
}

std::string sha256_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) {
        throw std::runtime_error("Cannot open file for hashing: " + path.string());
    }

    SHA256State state;
    std::array<uint8_t, 8192> buf{};
    while (in.read(reinterpret_cast<char*>(buf.data()), buf.size()) || in.gcount() > 0) {
        state.update(buf.data(), static_cast<size_t>(in.gcount()));
    }
    return state.finalize();
}

std::string compute_asset_hash(
    const std::filesystem::path& source_path,
    const std::string& config_yaml)
{
    std::ifstream in(source_path, std::ios::binary);
    if (!in.is_open()) {
        throw std::runtime_error("Cannot open file for hashing: " + source_path.string());
    }

    SHA256State state;

    // Hash file contents
    std::array<uint8_t, 8192> buf{};
    while (in.read(reinterpret_cast<char*>(buf.data()), buf.size()) || in.gcount() > 0) {
        state.update(buf.data(), static_cast<size_t>(in.gcount()));
    }

    // Separator + config
    const std::string sep = "||CONFIG||";
    state.update(reinterpret_cast<const uint8_t*>(sep.data()), sep.size());
    state.update(reinterpret_cast<const uint8_t*>(config_yaml.data()), config_yaml.size());

    return state.finalize();
}

}  // namespace simforge
