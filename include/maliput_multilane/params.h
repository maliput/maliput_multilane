// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
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
// Copyright 2021 Toyota Research Institute
#pragma once

#include <memory>

namespace maliput {
namespace multilane {
namespace params {

/// @defgroup road_network_configuration_builder_keys RoadNetwork configuration builder keys
///
/// Parameters used during the RoadNetwork building process.
///
/// When parameters are omitted the default value will be used.
///
/// Example of use:
/// @code{cpp}
/// #include <maliput_multilane/params.h>
/// #include <maliput_multilane/road_network_builder.h>
/// // ...
/// using namespace maliput::multilane;
/// const std::map<std::string, std::string> builder_configuration {
///   {params::kYamlFile, "/home/user/road_network.yaml"},
/// };
/// auto road_network = BuildRoadNetwork(RoadNetworkConfiguration::FromMap(builder_configuration))();
/// @endcode
///
/// @{

/// Path to a YAML description file.
///   - Default: ""
static constexpr char const* kYamlFile{"yaml_file"};

/// Serialized YAML description.
///   - Default: ""
static constexpr char const* kYamlDescription{"yaml_description"};

/// @}

/// @defgroup on_ramp_configuration_keys On-Ramp RoadNetwork configuration builder keys
///
/// Parameters used during the On-Ramp RoadNetwork building process.
///
/// When parameters are omitted the default value will be used.
///
/// Example of use:
/// @code{cpp}
/// #include <maliput_multilane/params.h>
/// #include <maliput_multilane/road_network_builder.h>
/// // ...
/// using namespace maliput::multilane;
/// const std::map<std::string, std::string> builder_configuration {
///   {maliput::multilane::kLaneWidth, 4.},
///   {maliput::multilane::kLeftShoulder, 2.},
///   {maliput::multilane::kRightShoulder, 2.},
///   {maliput::multilane::kLaneNumber, 1},
/// };
/// auto road_network = BuildOnRampMergeRoadNetwork(MultilaneRoadCharacteristics::FromMap(builder_configuration))();
/// @endcode
///
/// @{

/// Width of the lane.
///   - Default: "4."
static constexpr char const* kLaneWidth{"lane_width"};

/// Width of the left shoulder.
///   - Default: "2."
static constexpr char const* kLeftShoulder{"left_shoulder"};

/// Width of the right shoulder.
///   - Default: "2."
static constexpr char const* kRightShoulder{"right_shoulder"};

/// Number of lanes.
///   - Default: "1"
static constexpr char const* kLaneNumber{"lane_number"};

/// Elevation boundaries of the lane's space.
/// Being the first value the minimum elevation and the second value the maximum elevation.
///   - Default: @e "{0., 5.2}"
static constexpr char const* kElevationBounds{"elevation_bounds"};

/// @}

}  // namespace params
}  // namespace multilane
}  // namespace maliput
