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
#pragma once

#include <cmath>
#include <memory>

#include <maliput/api/road_geometry.h>
#include <maliput/common/maliput_copyable.h>

#include "maliput_multilane/builder.h"
#include "maliput_multilane/computation_policy.h"

namespace maliput {
namespace multilane {

/// MultilaneRoadCharacteristics computes and stores characteristics of a
/// multilane road network; i.e. bounds on the lane width and shoulders width.
/// Default settings are taken if no others are specified.
struct MultilaneRoadCharacteristics {
  /// Constructor for using default road geometries.
  MultilaneRoadCharacteristics() = default;

  /// Constructor for custom road geometries.
  ///
  /// @param lw Lane's width.
  /// @param lshoulder The left shoulder width.
  /// @param rshoulder The right shoulder width.
  /// @param lnumber The number of lanes.
  MultilaneRoadCharacteristics(double lw, double lshoulder, double rshoulder, int lnumber)
      : lane_width(lw), left_shoulder(lshoulder), right_shoulder(rshoulder), lane_number(lnumber) {}

  const double lane_width{4.};
  const double left_shoulder{2.};
  const double right_shoulder{2.};
  const int lane_number{1};

  const maliput::api::HBounds elevation_bounds{0., 5.2};
};

/// MultilaneOnrampMerge contains an example lane-merge scenario expressed as a
/// maliput mulitilane road geometry.  The intent of this class is to enable
/// easy creation and modification of road geometries for simulating/analyzing
/// such scenarios.
///
/// Implements the following onramp example, where each road section is composed
/// of sequences of linear and arc primitives:
///
/// <pre>
///           pre-merge           post-merge
///             road                 road
///        |------>------------+------>-------|
///        |------>------------+------>-------|
///                           /+------>-------|
///                          //
///                         //
///                        //   onramp
///                       ^^
///                       ||
///                       ||
///                       __
/// </pre>
///
/// The number of lanes of each road depends on the properties the
/// MultilaneRoadCharacteristics. `post` roads will have the full number of
/// lanes. `pre` and `onramp` roads will have half plus one lanes (note the
/// integer division) and will be placed to the left and right sides of `post`
/// road respectively. When the full lane number is even, two lanes from `pre`
/// and `onramp` will overlap. Otherwise, only one lane will overlap.
///
/// Note that this factory sets some constants to the `multilane::Builder` when
/// creating the RoadGeometry. Linear and angular tolerances, the scale length
/// and the ComputationPolicy are set to appropriate values to build this
/// RoadGeometry.
class MultilaneOnrampMerge {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MultilaneOnrampMerge)

  /// Constructor for the factory.
  ///
  /// @param rc A structure that aggregates the road boundary data.
  explicit MultilaneOnrampMerge(const MultilaneRoadCharacteristics& rc) : rc_(rc) {}

  /// Constructor for the example, using default MultilaneRoadCharacteristics
  /// settings.
  MultilaneOnrampMerge() : MultilaneOnrampMerge(MultilaneRoadCharacteristics{}) {}

  /// @return A std::unique_ptr<const maliput::api::RoadGeometry> with the
  /// onramp example.
  std::unique_ptr<const maliput::api::RoadGeometry> BuildOnramp() const;

 private:
  // Tolerances and properties for multilane's Builder.
  const double linear_tolerance_{0.01};
  const double angular_tolerance_{0.01 * M_PI};
  const double scale_length_{1.};
  const maliput::multilane::ComputationPolicy computation_policy_{maliput::multilane::ComputationPolicy::kPreferSpeed};
  // Road characteristics.
  const MultilaneRoadCharacteristics rc_;
};

}  // namespace multilane
}  // namespace maliput
