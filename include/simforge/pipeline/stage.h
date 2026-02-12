#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "simforge/core/types.h"

namespace simforge {

// ─── Result type ───────────────────────────────────────────────────

struct StageError {
    std::string stage_name;
    std::string asset_id;
    std::string message;
};

template <typename T>
class Result {
public:
    static Result ok(T value)           { return Result(std::move(value), {}); }
    static Result err(StageError error) { return Result({}, std::move(error)); }
    static Result err(StageError error, T partial) {
        return Result(std::move(partial), std::move(error));
    }

    [[nodiscard]] bool              is_ok()    const { return !error_.has_value(); }
    [[nodiscard]] bool              is_err()   const { return error_.has_value(); }
    [[nodiscard]] const T&          value()    const { return value_; }
    [[nodiscard]] T&                value()          { return value_; }
    [[nodiscard]] const StageError& error()    const { return error_.value(); }

private:
    Result(T val, std::optional<StageError> err)
        : value_(std::move(val)), error_(std::move(err)) {}

    T                           value_;
    std::optional<StageError>   error_;
};

// ─── Stage Interface ───────────────────────────────────────────────

/// Base class for all pipeline stages.
/// Each stage transforms an Asset in place, moving it from one
/// AssetStatus to the next.
class Stage {
public:
    virtual ~Stage() = default;

    /// Human-readable name for logging/reports.
    [[nodiscard]] virtual std::string name() const = 0;

    /// Configure this stage from a YAML node.
    /// Called once during pipeline construction.
    virtual void configure(const YAML::Node& config) = 0;

    /// Process a single asset. Returns the modified asset or an error.
    /// Implementations should be idempotent where possible.
    virtual Result<Asset> process(Asset asset) = 0;

    /// Optional: check whether this stage should run for a given asset.
    /// Default: always run.
    [[nodiscard]] virtual bool should_run(const Asset& asset) const {
        (void)asset;
        return true;
    }
};

using StagePtr = std::unique_ptr<Stage>;

// ─── Stage Registry ────────────────────────────────────────────────

/// Factory function signature for stage creation.
using StageFactory = std::function<StagePtr()>;

/// Global registry of available stages.
/// Adapters register themselves here at static init time.
class StageRegistry {
public:
    static StageRegistry& instance();

    void        register_stage(const std::string& name, StageFactory factory);
    StagePtr    create(const std::string& name) const;
    [[nodiscard]] std::vector<std::string> available() const;
    [[nodiscard]] bool has(const std::string& name) const;

private:
    StageRegistry() = default;
    std::unordered_map<std::string, StageFactory> factories_;
};

/// Helper macro for auto-registration of stages.
#define SIMFORGE_REGISTER_STAGE(ClassName, StageName)                         \
    namespace {                                                                \
    static const bool _reg_##ClassName = [] {                                 \
        ::simforge::StageRegistry::instance().register_stage(                 \
            StageName, [] { return std::make_unique<ClassName>(); });          \
        return true;                                                           \
    }();                                                                       \
    }

}  // namespace simforge
