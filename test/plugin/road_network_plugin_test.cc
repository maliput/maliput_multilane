// Copyright 2021 Toyota Research Institute
#include <map>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "maliput/common/filesystem.h"
#include "maliput/plugin/maliput_plugin.h"
#include "maliput/plugin/maliput_plugin_manager.h"
#include "maliput/plugin/maliput_plugin_type.h"
#include "maliput/plugin/road_network_loader.h"

#include "maliput_multilane/road_geometry.h"

namespace maliput {
namespace multilane {
namespace {

GTEST_TEST(RoadNetworkLoader, VerifyRoadNetworkPlugin) {
  // Get absolute path to a yaml file to be used for testing.
  const std::string MULTILANE_RESOURCE_ROOT{"MULTILANE_RESOURCE_ROOT"};
  const std::string kFileName{"/2x2_intersection.yaml"};
  const std::string env_path = maliput::common::Filesystem::get_env_path(MULTILANE_RESOURCE_ROOT);
  ASSERT_TRUE(!env_path.empty());

  // RoadNetworkLoader plugin id.
  const plugin::MaliputPlugin::Id kMultilanePluginId{"maliput_multilane"};
  // Multilane properties needed for loading a road geometry.
  const std::map<std::string, std::string> rg_multilane_properties{{"yaml_file", env_path + kFileName}};

  // Check MaliputPlugin existence.
  plugin::MaliputPluginManager manager{};
  const plugin::MaliputPlugin* rn_plugin{manager.GetPlugin(kMultilanePluginId)};
  ASSERT_NE(nullptr, rn_plugin);

  // Check multilane plugin is obtained.
  std::unique_ptr<maliput::plugin::RoadNetworkLoader> rn_loader;
  EXPECT_EQ(kMultilanePluginId.string(), rn_plugin->GetId());
  EXPECT_EQ(plugin::MaliputPluginType::kRoadNetworkLoader, rn_plugin->GetType());
  EXPECT_NO_THROW(rn_loader = rn_plugin->ExecuteSymbol<std::unique_ptr<plugin::RoadNetworkLoader>>(
                      plugin::RoadNetworkLoader::GetEntryPoint()));
  ASSERT_NE(nullptr, rn_loader);

  // Check multilane RoadNetwork is constructible.
  std::unique_ptr<const maliput::api::RoadNetwork> rn;
  EXPECT_NO_THROW(rn = (*rn_loader)(rg_multilane_properties));
  ASSERT_NE(nullptr, rn);
  auto multilane_rg = dynamic_cast<const RoadGeometry*>(rn->road_geometry());
  ASSERT_NE(nullptr, multilane_rg);
}

}  // namespace
}  // namespace multilane
}  // namespace maliput
