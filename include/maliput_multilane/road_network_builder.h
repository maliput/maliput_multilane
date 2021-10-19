// Copyright 2021 Toyota Research Institute
#pragma once

#include <memory>

#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>

#include "maliput_multilane/multilane_onramp_merge.h"

namespace maliput {
namespace multilane {

/// Contains the attributes needed for building a api::RoadNetwork.
struct RoadNetworkConfiguration {
  /// Path to a YAML description file.
  std::string yaml_file{""};
  /// Serialized YAML description.
  std::string yaml_description{""};
};

/// Builds an api::RoadNetwork based on multilane implementation.
/// @param road_network_configuration Holds the properties to build the RoadNetwork.
///                                   When `road_network_configuration.yaml_file` is empty,
///                                   `road_network_configuration.yaml_description` will be used instead.
/// @return A maliput::api::RoadNetwork.
/// @throws maliput::common::assertion_error When both `road_network_configuration.yaml_file` and
/// `road_network_configuration.yaml_description` are empty.
std::unique_ptr<api::RoadNetwork> BuildRoadNetwork(const RoadNetworkConfiguration& road_network_configuration);

/// Builds an api::RoadNetwork based on MultilaneOnrampMerge implementation.
/// Rulebook and related entities are only set to correctly build the RoadNetwork but they are expected to be empty.
/// @param road_characteristics Holds the properties to build a MultilaneOnrampMerge.
/// @return A maliput::api::RoadNetwork.
std::unique_ptr<api::RoadNetwork> BuildOnRampMergeRoadNetwork(const MultilaneRoadCharacteristics& road_characteristics);

}  // namespace multilane
}  // namespace maliput
