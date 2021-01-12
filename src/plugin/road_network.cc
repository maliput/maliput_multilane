// Copyright 2021 Toyota Research Institute
#include <memory>

#include "maliput/plugin/road_network_loader.h"
#include "maliput_multilane/road_network_builder.h"

namespace maliput {
namespace multilane {
namespace plugin {
namespace {

maliput::multilane::RoadNetworkConfiguration GetPropertiesFromStringMap(
    const std::map<std::string, std::string>& parameters) {
  std::string yaml_file;
  auto it = parameters.find("yaml_file");
  if (it != parameters.end()) {
    yaml_file = it->second;
  }
  return {yaml_file};
}

class RoadNetwork : public maliput::plugin::RoadNetworkLoader {
 public:
  std::unique_ptr<const maliput::api::RoadNetwork> operator()(
      const std::map<std::string, std::string>& properties) const override {
    return maliput::multilane::BuildRoadNetwork(GetPropertiesFromStringMap(properties));
  }
};

}  // namespace

REGISTER_ROAD_NETWORK_LOADER_PLUGIN("maliput_multilane", RoadNetwork);

}  // namespace plugin
}  // namespace multilane
}  // namespace maliput
