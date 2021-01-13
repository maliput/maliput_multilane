// Copyright 2021 Toyota Research Institute
#include <memory>

#include "maliput/plugin/road_network_loader.h"
#include "maliput_multilane/road_network_builder.h"

namespace maliput {
namespace multilane {
namespace plugin {
namespace {

// Return a multilane::RoadNetworkConfiguration object out of a map of strings.
// @param parameters  A dictionary of properties to fill in a multilane::RoadNetworkConfiguration struct.
//                    Keys are the names of attributes in multilane::RoadNetworkConfiguration.
// @returns A multilane::RoadNetworkConfiguration.
maliput::multilane::RoadNetworkConfiguration GetPropertiesFromStringMap(
    const std::map<std::string, std::string>& parameters) {
  auto it = parameters.find("yaml_file");
  const std::string yaml_file = it != parameters.end() ? it->second : "";
  return {yaml_file};
}

// Implementation of a maliput::plugin::RoadNetworkLoader using multilane backend.
class RoadNetworkLoader : public maliput::plugin::RoadNetworkLoader {
 public:
  std::unique_ptr<const maliput::api::RoadNetwork> operator()(
      const std::map<std::string, std::string>& properties) const override {
    return maliput::multilane::BuildRoadNetwork(GetPropertiesFromStringMap(properties));
  }
};

}  // namespace

REGISTER_ROAD_NETWORK_LOADER_PLUGIN("maliput_multilane", RoadNetworkLoader);

}  // namespace plugin
}  // namespace multilane
}  // namespace maliput
