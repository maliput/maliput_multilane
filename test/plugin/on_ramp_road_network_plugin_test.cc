// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2021-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

// Tests RoadNetworkLoader implementation for the OnRamp builder.
class OnRampRoadNetworkLoaderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    setenv("MALIPUT_PLUGIN_PATH", DEF_ROAD_NETWORK_PLUGIN, 1);

    ASSERT_TRUE(!resource_path_.empty());
    manager_ = std::make_unique<maliput::plugin::MaliputPluginManager>();
    // Check MaliputPlugin existence.
    const plugin::MaliputPlugin* rn_plugin{manager_->GetPlugin(kMultilanePluginId)};
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

  // RoadNetworkLoader plugin id.
  const plugin::MaliputPlugin::Id kMultilanePluginId{"maliput_multilane_on_ramp"};
  const std::string resource_path_{DEF_MULTILANE_RESOURCE_ROOT};
  std::unique_ptr<maliput::plugin::MaliputPluginManager> manager_;
  std::unique_ptr<maliput::plugin::RoadNetworkLoader> rn_loader_{nullptr};
};

// Tests loading a RoadGeometry using the MultilaneOnRampMerge builder.
TEST_F(OnRampRoadNetworkLoaderTest, UsingOnRampBuilder) {
  // Multilane properties.
  const std::map<std::string, std::string> rg_multilane_properties{
      {"lane_width", "4."}, {"left_shoulder", "2."},           {"right_shoulder", "2."},
      {"lane_number", "1"}, {"elevation_bounds", "{0., 7.5}"},
  };
  CheckMultilaneRoadNetworkIsConstructible(rg_multilane_properties);
}

}  // namespace
}  // namespace multilane
}  // namespace maliput
