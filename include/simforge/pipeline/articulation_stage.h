// About: Pipeline stage that merges articulation data from source files,
// YAML config, and sidecar metadata into Asset.kinematic_tree.
#pragma once

#include "simforge/pipeline/stage.h"

namespace simforge::stages {

class ArticulationStage : public Stage {
public:
    [[nodiscard]] std::string name() const override { return "articulation"; }

    void configure(const YAML::Node& config) override;
    Result<Asset> process(Asset asset) override;

    [[nodiscard]] bool should_run(const Asset& asset) const override {
        return asset.status == AssetStatus::Ingested;
    }

private:
    YAML::Node config_;
    bool       load_sidecars_{true};

    /// Load sidecar file (.simforge.yaml or .simforge.json) if present.
    void load_sidecar(Asset& asset) const;

    /// Parse articulation block from YAML config into the tree.
    void apply_yaml_overlay(KinematicTree& tree) const;

    /// Parse a YAML links/joints/actuators/sensors block into a KinematicTree.
    static KinematicTree parse_tree_from_yaml(const YAML::Node& node);
    static Link parse_link_from_yaml(const YAML::Node& node);
    static Joint parse_joint_from_yaml(const YAML::Node& node);
    static Actuator parse_actuator_from_yaml(const YAML::Node& node);
    static Sensor parse_sensor_from_yaml(const YAML::Node& node);

    /// Merge source tree with overlay. Overlay values take priority.
    static void merge_trees(KinematicTree& base, const KinematicTree& overlay);
};

}  // namespace simforge::stages
