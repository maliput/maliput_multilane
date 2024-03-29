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
#include "maliput_multilane/multilane_onramp_merge.h"

#include <maliput/api/road_geometry.h>
#include <maliput/math/vector.h>

#include "maliput_multilane/params.h"

namespace maliput {
namespace multilane {

using ::maliput::api::LaneEnd;

MultilaneRoadCharacteristics MultilaneRoadCharacteristics::FromMap(
    const std::map<std::string, std::string>& road_characteristics) {
  MultilaneRoadCharacteristics config;
  auto it = road_characteristics.find(params::kLaneNumber);
  if (it != road_characteristics.end()) {
    config.lane_number = std::stoi(it->second);
  }
  it = road_characteristics.find(params::kLaneWidth);
  if (it != road_characteristics.end()) {
    config.lane_width = std::stod(it->second);
  }
  it = road_characteristics.find(params::kLeftShoulder);
  if (it != road_characteristics.end()) {
    config.left_shoulder = std::stod(it->second);
  }
  it = road_characteristics.find(params::kRightShoulder);
  if (it != road_characteristics.end()) {
    config.right_shoulder = std::stod(it->second);
  }
  it = road_characteristics.find(params::kElevationBounds);
  if (it != road_characteristics.end()) {
    const maliput::math::Vector2 h_bounds = maliput::math::Vector2::FromStr(it->second);
    config.elevation_bounds = maliput::api::HBounds(h_bounds[0], h_bounds[1]);
  }
  return config;
}

std::map<std::string, std::string> MultilaneRoadCharacteristics::ToStringMap() const {
  return {{params::kLaneWidth, std::to_string(lane_width)},
          {params::kLeftShoulder, std::to_string(left_shoulder)},
          {params::kRightShoulder, std::to_string(right_shoulder)},
          {params::kLaneNumber, std::to_string(lane_number)},
          {params::kElevationBounds, maliput::math::Vector2(elevation_bounds.min(), elevation_bounds.max()).to_str()}};
}

std::unique_ptr<const maliput::api::RoadGeometry> MultilaneOnrampMerge::BuildOnramp() const {
  auto rb = BuilderFactory().Make(rc_.lane_width, rc_.elevation_bounds, linear_tolerance_, angular_tolerance_,
                                  scale_length_, computation_policy_);

  // Initialize roads lane layouts.
  // Reference lane from which the reference curve of the segment is placed (at
  // kRefR0 lateral distance).
  const int kRefLane = 0;
  // Distance between the reference curve and kRefLane lane curve.
  const double kRefR0 = 0.;
  const LaneLayout lane_layout(rc_.left_shoulder, rc_.right_shoulder, rc_.lane_number, kRefLane, kRefR0);

  // Initialize the road from the origin.
  const EndpointXy kOriginXy{0., 0., 0.};
  const EndpointZ kFlatZ{0., 0., 0., {}};
  const Endpoint kRoadOrigin{kOriginXy, kFlatZ};

  // Construct the post-merge road.
  const double kPostArcLength = 25.;
  const double kPostArcRadius = 40.;
  const auto& post5 = rb->Connect("post5", lane_layout, StartReference().at(kRoadOrigin, Direction::kForward),
                                  ArcOffset(kPostArcLength, -kPostArcRadius / kPostArcLength),
                                  EndReference().z_at(kFlatZ, Direction::kForward));
  const auto& post4 = rb->Connect(
      "post4", lane_layout, StartReference().at(*post5, LaneEnd::kFinish, Direction::kForward),
      ArcOffset(kPostArcLength, kPostArcRadius / kPostArcLength), EndReference().z_at(kFlatZ, Direction::kForward));
  const auto& post3 = rb->Connect(
      "post3", lane_layout, StartReference().at(*post4, LaneEnd::kFinish, Direction::kForward),
      ArcOffset(kPostArcLength, -kPostArcRadius / kPostArcLength), EndReference().z_at(kFlatZ, Direction::kForward));
  const auto& post2 = rb->Connect(
      "post2", lane_layout, StartReference().at(*post3, LaneEnd::kFinish, Direction::kForward),
      ArcOffset(kPostArcLength, kPostArcRadius / kPostArcLength), EndReference().z_at(kFlatZ, Direction::kForward));
  const auto& post1 = rb->Connect(
      "post1", lane_layout, StartReference().at(*post2, LaneEnd::kFinish, Direction::kForward),
      ArcOffset(kPostArcLength, -kPostArcRadius / kPostArcLength), EndReference().z_at(kFlatZ, Direction::kForward));
  const auto& post0 = rb->Connect(
      "post0", lane_layout, StartReference().at(*post1, LaneEnd::kFinish, Direction::kForward),
      ArcOffset(kPostArcLength, kPostArcRadius / kPostArcLength), EndReference().z_at(kFlatZ, Direction::kForward));

  // Construct the pre-merge road.
  const double kPostLinearLength = 100.;
  const int pre_num_lanes = rc_.lane_number / 2 + 1;
  const LaneLayout pre_lane_layout(rc_.left_shoulder, rc_.right_shoulder, pre_num_lanes, kRefLane, kRefR0);
  const auto& pre0 =
      rb->Connect("pre0", pre_lane_layout, StartReference().at(*post0, LaneEnd::kFinish, Direction::kForward),
                  LineOffset(kPostLinearLength), EndReference().z_at(kFlatZ, Direction::kForward));

  // Construct the on-ramp (starting at merge junction at the `pre0` - `post0`
  // interface and working backwards).
  const double kOnrampArcLength = 35.;
  const double kOnrampArcRadius = 50.;
  const double kOnrampLinearLength = 100.;
  const int onramp_num_lanes = rc_.lane_number / 2 + 1;
  const LaneLayout on_ramp_lane_layout(
      rc_.left_shoulder, rc_.right_shoulder, onramp_num_lanes, kRefLane,
      kRefR0 + static_cast<double>(rc_.lane_number - onramp_num_lanes) * rc_.lane_width);
  const auto& onramp1 =
      rb->Connect("onramp1", on_ramp_lane_layout, StartReference().at(*post0, LaneEnd::kFinish, Direction::kForward),
                  ArcOffset(kOnrampArcLength, kOnrampArcRadius / kOnrampArcLength),
                  EndReference().z_at(kFlatZ, Direction::kForward));
  const auto& onramp0 =
      rb->Connect("onramp0", on_ramp_lane_layout, StartReference().at(*onramp1, LaneEnd::kFinish, Direction::kForward),
                  LineOffset(kOnrampLinearLength), EndReference().z_at(kFlatZ, Direction::kForward));

  // Manually specify the default branches for all junctions in the road.
  auto set_default_branches = [&](const Connection* in, int in_first_lane, const Connection* out, int out_first_lane,
                                  int num_lanes) {
    for (int i = 0; i < num_lanes; ++i) {
      rb->SetDefaultBranch(in, i + in_first_lane, LaneEnd::kStart, out, i + out_first_lane, LaneEnd::kFinish);
    }
  };
  set_default_branches(pre0, 0, post0, 0, pre_num_lanes);
  set_default_branches(post0, 0, post1, 0, rc_.lane_number);
  set_default_branches(post1, 0, post2, 0, rc_.lane_number);
  set_default_branches(post2, 0, post3, 0, rc_.lane_number);
  set_default_branches(post3, 0, post4, 0, rc_.lane_number);
  set_default_branches(post4, 0, post5, 0, rc_.lane_number);
  set_default_branches(onramp1, 0, post0, rc_.lane_number / 2 - (rc_.lane_number % 2 == 0 ? 1 : 0), onramp_num_lanes);
  set_default_branches(onramp0, 0, onramp1, 0, onramp_num_lanes);

  return rb->Build(maliput::api::RoadGeometryId{"multilane-merge-example"});
}

}  // namespace multilane
}  // namespace maliput
