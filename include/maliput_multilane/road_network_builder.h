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
#pragma once

#include <memory>

#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>

#include "maliput_multilane/multilane_onramp_merge.h"

namespace maliput {
namespace multilane {

/// Contains the attributes needed for building a api::RoadNetwork.
struct RoadNetworkConfiguration {
  /// Creates a RoadNetworkConfiguration out of a string dictionary.
  /// @details The keys of the map are listed at @ref road_network_configuration_builder_keys.
  /// @param road_network_configuration A string-string map containing the configuration for the builder.
  static RoadNetworkConfiguration FromMap(const std::map<std::string, std::string>& road_network_configuration);

  /// @details The keys of the map are listed at @ref road_network_configuration_builder_keys.
  /// @returns A string-string map containing the RoadGeometry configuration.
  std::map<std::string, std::string> ToStringMap() const;

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
