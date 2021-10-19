// Copyright 2021 Toyota Research Institute
#include <memory>

#include <maliput/common/maliput_abort.h>
#include <maliput/plugin/road_network_loader.h>

#include "maliput_multilane/multilane_onramp_merge.h"
#include "maliput_multilane/road_network_builder.h"

namespace maliput {
namespace multilane {
namespace plugin {
namespace {

// Return a multilane::RoadNetworkConfiguration object out of a map of strings.
// @param parameters  A dictionary of properties to fill in a multilane::RoadNetworkConfiguration struct.
//                    Keys are the names of attributes in multilane::RoadNetworkConfiguration.
// @returns A multilane::RoadNetworkConfiguration.
maliput::multilane::RoadNetworkConfiguration GetRoadNetworkPropertiesFromStringMap(
    const std::map<std::string, std::string>& parameters) {
  auto it = parameters.find("yaml_file");
  const std::string yaml_file = it != parameters.end() ? it->second : "";

  it = parameters.find("yaml_description");
  const std::string yaml_description = it != parameters.end() ? it->second : "";
  return {yaml_file, yaml_description};
}

// Return a multilane::MultilaneRoadCharacteristics object out of a map of strings.
// @param parameters  A dictionary of properties to fill in a multilane::MultilaneRoadCharacteristics struct.
//                    Keys are the names of attributes in multilane::MultilaneRoadCharacteristics.
// @returns A multilane::MultilaneRoadCharacteristics.
maliput::multilane::MultilaneRoadCharacteristics GetOnRampPropertiesFromStringMap(
    const std::map<std::string, std::string>& parameters) {
  MultilaneRoadCharacteristics default_on_ramp_params{};
  auto it = parameters.find("lane_width");
  const double lane_width = it != parameters.end() ? std::stod(it->second) : default_on_ramp_params.lane_width;

  it = parameters.find("left_shoulder");
  const double left_shoulder = it != parameters.end() ? std::stod(it->second) : default_on_ramp_params.left_shoulder;

  it = parameters.find("right_shoulder");
  const double right_shoulder = it != parameters.end() ? std::stod(it->second) : default_on_ramp_params.right_shoulder;

  it = parameters.find("lane_number");
  const int lane_number = it != parameters.end() ? std::stoi(it->second) : default_on_ramp_params.lane_number;

  return {lane_width, left_shoulder, right_shoulder, lane_number};
}

// Builds a maliput::api::RoadNetwork depending on `properties`.
// A 'road_network_source' flag is used to differentiate which multilane implementation will be used:
//    1. The base multilane builder loaded from a yaml file or a serialized yaml description.
//    2. The MultilaneOnRampMerge builder.
std::unique_ptr<maliput::api::RoadNetwork> BuildRoadNetworkFromParams(
    const std::map<std::string, std::string>& properties) {
  auto it = properties.find("road_network_source");
  if (it == properties.end()) {
    MALIPUT_THROW_MESSAGE("'road_network_source' parameter is missing: 'yaml' or 'on_ramp_merge' should be selected.");
  }
  const std::string source = it->second;
  if (source == "yaml") {
    return maliput::multilane::BuildRoadNetwork(GetRoadNetworkPropertiesFromStringMap(properties));
  } else if (source == "on_ramp_merge") {
    return maliput::multilane::BuildOnRampMergeRoadNetwork(GetOnRampPropertiesFromStringMap(properties));
  } else {
    MALIPUT_THROW_MESSAGE(
        "'road_network_source' parameter doesn't contain a valid source: 'yaml' or 'on_ramp_merge' should be "
        "selected.");
  }
}

// Implementation of a maliput::plugin::RoadNetworkLoader using multilane backend.
class RoadNetworkLoader : public maliput::plugin::RoadNetworkLoader {
 public:
  std::unique_ptr<maliput::api::RoadNetwork> operator()(
      const std::map<std::string, std::string>& properties) const override {
    return BuildRoadNetworkFromParams(properties);
  }
};

}  // namespace

REGISTER_ROAD_NETWORK_LOADER_PLUGIN("maliput_multilane", RoadNetworkLoader);

}  // namespace plugin
}  // namespace multilane
}  // namespace maliput
