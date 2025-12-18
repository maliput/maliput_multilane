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

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include <maliput/api/junction.h>
#include <maliput/api/lane.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/segment.h>
#include <maliput/common/maliput_abort.h>
#include <maliput/common/maliput_copyable.h>

#include "maliput_multilane/cubic_polynomial.h"
#include "maliput_multilane/lane.h"
#include "maliput_multilane/road_curve.h"

namespace maliput {
namespace multilane {

class ArcLane;
class LineLane;

/// An api::Segment implementation.
class Segment : public api::Segment {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Segment);

  /// Constructs a new Segment.
  ///
  /// The Segment is not fully initialized until NewLane() is called at least
  /// once. `junction` must remain valid for the lifetime of this class.
  /// @param id Segment's ID.
  /// @param junction Parent junction.
  /// @param register_lane will be called on any new Lane instance created as
  /// a child of the Segment.
  /// @param road_curve A curve that defines the reference trajectory over the
  /// segment. A child Lane object will be constructed from an offset of the
  /// road_curve's reference curve.
  /// @param r_min Lateral distance to the minimum extent of road_curve's curve
  /// from where Segment's surface starts. It must be smaller or equal than
  /// `r_max`.
  /// @param r_max Lateral distance to the maximum extent of road_curve's curve
  /// from where Segment's surface ends. It should be greater or equal than
  /// `r_min`.
  /// @param elevation_bounds The height bounds over the segment' surface.
  Segment(const api::SegmentId& id, const api::Junction* junction,
          const std::function<void(const api::Lane*)>& register_lane, std::unique_ptr<RoadCurve> road_curve,
          double r_min, double r_max, const api::HBounds& elevation_bounds)
      : id_(id),
        junction_(junction),
        register_lane_(register_lane),
        road_curve_(std::move(road_curve)),
        r_min_(r_min),
        r_max_(r_max),
        elevation_bounds_(elevation_bounds) {
    MALIPUT_DEMAND(road_curve_.get() != nullptr);
    MALIPUT_DEMAND(r_min <= r_max);
    MALIPUT_DEMAND(road_curve_->IsValid(r_min_, r_max_, elevation_bounds_));
    MALIPUT_DEMAND(junction_->road_geometry()->linear_tolerance() == road_curve_->linear_tolerance());
  }

  /// Creates a new Lane object.
  ///
  /// Driveable bounds of the lane will be derived based on the lateral offset
  /// of it so as to reach `r_min` and `r_max` distances (see class constructor
  /// for more details).
  /// @param id Lane's ID.
  /// @param r0 Lateral displacement of the Lane with respect to segment
  /// RoadCurve's reference curve. It must be greater than `r_min` and smaller
  /// than `r_max`, and be greater than the last lane's r0 displacement (if
  /// any).
  /// @param lane_bounds Nominal bounds of the lane, uniform along the entire
  /// reference path. It must fit inside segments bounds when those are
  /// translated to `r0` offset distance.
  /// @return A Lane object.
  Lane* NewLane(api::LaneId id, double r0, const api::RBounds& lane_bounds);

  ~Segment() override = default;

 private:
  api::SegmentId do_id() const override { return id_; }

  const api::Junction* do_junction() const override;

  int do_num_lanes() const override { return lanes_.size(); }

  const api::Lane* do_lane(int index) const override;

  const api::LaneBoundary* do_boundary(int index) const override { return nullptr; }

  // Segment's ID.
  api::SegmentId id_;
  // Parent junction.
  const api::Junction* junction_{};
  std::function<void(const api::Lane*)> register_lane_;
  // Child Lane vector.
  std::vector<std::unique_ptr<Lane>> lanes_;
  // Reference trajectory over the Segment's surface.
  std::unique_ptr<RoadCurve> road_curve_;
  // Lateral distance to the minimum extent of road_curve_'s curve from where
  // Segment's surface starts.
  const double r_min_{};
  // Lateral distance to the maximum extent of road_curve_'s curve from where
  // Segment's surface ends.
  const double r_max_{};
  // Elevation bounds over the Segment's surface.
  const api::HBounds elevation_bounds_;
};

}  // namespace multilane
}  // namespace maliput
