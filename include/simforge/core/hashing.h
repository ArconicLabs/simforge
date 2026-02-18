// About: SHA-256 hashing utilities for incremental processing — computes
// content hashes of source files and stage configs to detect changes.
#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace simforge {

/// Compute SHA-256 hex digest of raw bytes.
[[nodiscard]] std::string sha256_bytes(const uint8_t* data, size_t len);

/// Compute SHA-256 hex digest of a string.
[[nodiscard]] std::string sha256_string(const std::string& input);

/// Compute SHA-256 hex digest of a file's contents.
[[nodiscard]] std::string sha256_file(const std::filesystem::path& path);

/// Compute a combined hash of source file + stage config for cache keying.
/// Hash input = file bytes + "||CONFIG||" + config_yaml.
[[nodiscard]] std::string compute_asset_hash(
    const std::filesystem::path& source_path,
    const std::string& config_yaml);

}  // namespace simforge
