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

#include <memory>
#include <vector>

#include <maliput/api/basic_id_index.h>
#include <maliput/api/branch_point.h>
#include <maliput/api/junction.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/road_geometry.h>
#include <maliput/common/maliput_copyable.h>
#include <maliput/math/vector.h>

#include "maliput_multilane/branch_point.h"
#include "maliput_multilane/junction.h"

namespace maliput {
namespace multilane {

/// A simple api::RoadGeometry implementation that only supports a single
/// lane per segment.  Use the Builder interface to actually assemble
/// a sensible road network.
class RoadGeometry : public api::RoadGeometry {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RoadGeometry)

  /// Constructs an empty RoadGeometry with the specified tolerances and
  /// scale-length.
  RoadGeometry(const api::RoadGeometryId& id, double linear_tolerance, double angular_tolerance, double scale_length)
      : id_(id),
        linear_tolerance_(linear_tolerance),
        angular_tolerance_(angular_tolerance),
        scale_length_(scale_length) {}

  /// Creates and adds a new Junction with the specified @p id.
  Junction* NewJunction(api::JunctionId id);

  /// Creates and adds a new BranchPoint with the specified @p id.
  BranchPoint* NewBranchPoint(api::BranchPointId id);

  ~RoadGeometry() override = default;

 private:
  api::RoadGeometryId do_id() const override { return id_; }

  int do_num_junctions() const override { return junctions_.size(); }

  const api::Junction* do_junction(int index) const override;

  int do_num_branch_points() const override { return branch_points_.size(); }

  const api::BranchPoint* do_branch_point(int index) const override;

  const IdIndex& DoById() const override { return id_index_; }

  // Returns a RoadPositionResult for a lane containing the provided
  // `inertial_position`.
  // Either if there is or not a containing lane, the position is returned for
  // the lane whose centerline curve is closest to `inertial_position`. In other
  // words, for the lane whose LanePosition makes the r coordinate be the
  // smallest.
  // When `hint` is provided, the search is restricted to the `hint->lane`
  // and lanes adjacent to `hint->lane`.
  // TODO(agalbachicar) Take into account `h` coordinate to return by minimum
  //                    `h` and then minimum `r`.
  api::RoadPositionResult DoToRoadPosition(const api::InertialPosition& inertial_position,
                                           const std::optional<api::RoadPosition>& hint) const override;

  // TODO(agalbachicar) Needs implementation.
  std::vector<api::RoadPositionResult> DoFindRoadPositions(const api::InertialPosition& inertial_position,
                                                           double radius) const override;

  double do_linear_tolerance() const override { return linear_tolerance_; }

  double do_angular_tolerance() const override { return angular_tolerance_; }

  // TODO(maddog@tri.global)  scale_length should really be kept consistent
  //                          with the geometry of the curves themselves,
  //                          perhaps even derived from the curves directly.
  double do_scale_length() const override { return scale_length_; }

  // TODO(#45) Allow builder and loaders to set this value.
  math::Vector3 do_inertial_to_backend_frame_translation() const override { return {0., 0., 0.}; }

  std::string DoBackendCustomCommand(const std::string& command) const override {
    MALIPUT_THROW_MESSAGE("Method not implemented by maliput_multilane.");
  }

  // TODO(slopez) Needs implementation.
  std::string DoGeoReferenceInfo() const override { return {}; }

  api::RoadGeometryId id_;
  double linear_tolerance_{};
  double angular_tolerance_{};
  double scale_length_{};
  std::vector<std::unique_ptr<Junction>> junctions_;
  std::vector<std::unique_ptr<BranchPoint>> branch_points_;
  api::BasicIdIndex id_index_;
};

}  // namespace multilane
}  // namespace maliput
