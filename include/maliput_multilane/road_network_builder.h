// Copyright 2021 Toyota Research Institute
#pragma once

#include "maliput/api/road_geometry.h"
#include "maliput/api/road_network.h"

#include <memory>

namespace maliput {
namespace multilane {

/// Contains the attributes needed for building a api::RoadNetwork.
struct RoadNetworkConfiguration {
  std::string yaml_file{""};
};

/// Builds an api::RoadNetwork based on multilane implementation.
/// @param road_network_configuration Holds the properties to build the RoadNetwork.
/// @return A maliput::api::RoadNetwork.
std::unique_ptr<const api::RoadNetwork> BuildRoadNetwork(const RoadNetworkConfiguration& road_network_configuration);

}  // namespace multilane
}  // namespace maliput
