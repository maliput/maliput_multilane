// Copyright 2021 Toyota Research Institute
#include <map>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <maliput/common/filesystem.h>
#include <maliput/plugin/maliput_plugin.h>
#include <maliput/plugin/maliput_plugin_manager.h>
#include <maliput/plugin/maliput_plugin_type.h>
#include <maliput/plugin/road_network_loader.h>

#include "maliput_multilane/road_geometry.h"

namespace maliput {
namespace multilane {
namespace {

// Tests RoadNetworkLoader plugin.
class RoadNetworkLoaderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_TRUE(!env_path_.empty());
    // Check MaliputPlugin existence.
    const plugin::MaliputPlugin* rn_plugin{manager_.GetPlugin(kMultilanePluginId)};
    ASSERT_NE(nullptr, rn_plugin);

    // Check multilane plugin is obtained.
    EXPECT_EQ(kMultilanePluginId.string(), rn_plugin->GetId());
    EXPECT_EQ(plugin::MaliputPluginType::kRoadNetworkLoader, rn_plugin->GetType());
    maliput::plugin::RoadNetworkLoaderPtr rn_loader_ptr{nullptr};
    ASSERT_NO_THROW(rn_loader_ptr = rn_plugin->ExecuteSymbol<plugin::RoadNetworkLoaderPtr>(
                        plugin::RoadNetworkLoader::GetEntryPoint()));
    ASSERT_NE(nullptr, rn_loader_ptr);
    rn_loader_.reset(reinterpret_cast<maliput::plugin::RoadNetworkLoader*>(rn_loader_ptr));
  }

  void CheckMultilaneRoadNetworkIsConstructible(const std::map<std::string, std::string>& rg_multilane_properties) {
    std::unique_ptr<const maliput::api::RoadNetwork> rn;
    (*rn_loader_)(rg_multilane_properties);
    ASSERT_NO_THROW(rn = (*rn_loader_)(rg_multilane_properties));
    ASSERT_NE(nullptr, rn);
    auto multilane_rg = dynamic_cast<const RoadGeometry*>(rn->road_geometry());
    ASSERT_NE(nullptr, multilane_rg);
  }

  const std::string MULTILANE_RESOURCE_ROOT{"MULTILANE_RESOURCE_ROOT"};
  // RoadNetworkLoader plugin id.
  const plugin::MaliputPlugin::Id kMultilanePluginId{"maliput_multilane"};

  const std::string env_path_ = maliput::common::Filesystem::get_env_path(MULTILANE_RESOURCE_ROOT);
  maliput::plugin::MaliputPluginManager manager_{};
  std::unique_ptr<maliput::plugin::RoadNetworkLoader> rn_loader_{nullptr};
};

// Tests loading a RoadGeometry using a yaml file.
TEST_F(RoadNetworkLoaderTest, UsingYamlFile) {
  // Get absolute path to a yaml file to be used for testing.
  const std::string kFileName{"/2x2_intersection.yaml"};
  // Multilane properties.
  const std::map<std::string, std::string> rg_multilane_properties{{"road_network_source", "yaml"},
                                                                   {"yaml_file", env_path_ + kFileName}};
  CheckMultilaneRoadNetworkIsConstructible(rg_multilane_properties);
}

// Tests loading a RoadGeometry using a serialized yaml description.
TEST_F(RoadNetworkLoaderTest, UsingYamlDescription) {
  // Yaml description based on "long_start_and_end_lanes" yaml file.
  const std::string kYamlDescription{R"R(
maliput_multilane_builder:
  id: "long_start_and_end_lanes"
  computation_policy: "prefer-accuracy"
  scale_length: 1
  lane_width: 6
  left_shoulder: 5
  right_shoulder: 5
  elevation_bounds: [0, 5]
  linear_tolerance: .01
  angular_tolerance: 0.5
  points:
    start:
      xypoint: [0, 0, 0]  # x,y, heading
      zpoint: [0, 0, 0, 0]  # z, z_dot, theta (super-elevation), theta_dot
  connections:
    0:
      lanes: [1, 0, 0]  # num_lanes, ref_lane, r_ref
      start: ["ref", "points.start.forward"]
      length: 1000
      z_end: ["ref", [0, 0, 0]]
    1:
      lanes: [1, 0, 0]
      start: ["ref", "connections.0.end.ref.forward"]
      length: 10
      z_end: ["ref", [0, 0, 0]]
    2:
      lanes: [1, 0, 0]
      start: ["ref", "connections.1.end.ref.forward"]
      length: 1000
      z_end: ["ref", [0, 0, 0]]
)R"};

  // Multilane properties needed for loading a road geometry.
  const std::map<std::string, std::string> rg_multilane_properties{{"road_network_source", "yaml"},
                                                                   {"yaml_description", kYamlDescription}};
  CheckMultilaneRoadNetworkIsConstructible(rg_multilane_properties);
}

// Tests loading a RoadGeometry using the MultilaneOnRampMerge builder.
TEST_F(RoadNetworkLoaderTest, UsingOnRampBuilder) {
  // Multilane properties.
  const std::map<std::string, std::string> rg_multilane_properties{
      {"road_network_source", "on_ramp_merge"},
      {"lane_width", "4."},
      {"left_shoulder", "2."},
      {"right_shoulder", "2."},
      {"lane_number", "1"},
  };
  CheckMultilaneRoadNetworkIsConstructible(rg_multilane_properties);
}

}  // namespace
}  // namespace multilane
}  // namespace maliput
