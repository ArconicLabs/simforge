#include "simforge/pipeline/stage.h"

#include <spdlog/spdlog.h>
#include <stdexcept>

namespace simforge {

StageRegistry& StageRegistry::instance() {
    static StageRegistry reg;
    return reg;
}

void StageRegistry::register_stage(const std::string& name, StageFactory factory) {
    spdlog::debug("Registering stage: {}", name);
    factories_[name] = std::move(factory);
}

StagePtr StageRegistry::create(const std::string& name) const {
    auto it = factories_.find(name);
    if (it == factories_.end()) {
        throw std::runtime_error("Unknown stage: " + name +
            ". Available: " + [&] {
                std::string s;
                for (const auto& [k, _] : factories_) s += k + " ";
                return s;
            }());
    }
    return it->second();
}

std::vector<std::string> StageRegistry::available() const {
    std::vector<std::string> names;
    names.reserve(factories_.size());
    for (const auto& [name, _] : factories_) {
        names.push_back(name);
    }
    return names;
}

bool StageRegistry::has(const std::string& name) const {
    return factories_.count(name) > 0;
}

}  // namespace simforge
