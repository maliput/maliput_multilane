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
#include <optional>

#include <maliput/api/branch_point.h>
#include <maliput/api/lane.h>
#include <maliput/api/segment.h>
#include <maliput/common/maliput_abort.h>
#include <maliput/common/maliput_copyable.h>

#include "maliput_multilane/cubic_polynomial.h"
#include "maliput_multilane/road_curve.h"

namespace maliput {
namespace multilane {

class BranchPoint;

/// Base class for the multilane implementation of api::Lane.
class Lane : public api::Lane {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Lane)

  /// Constructs a Lane.
  ///
  /// @param id the ID
  /// @param segment the Segment to which this Lane will belong, which must
  ///        remain valid for the lifetime of this class
  /// @param index Lane's index to identify it when querying parent @p segment.
  ///        It must be positive.
  /// @param lane_bounds nominal bounds of the lane, uniform along the entire
  ///        reference path, which must be a subset of @p segment_bounds
  /// @param segment_bounds segment bounds of the lane, uniform along the
  ///        entire reference path
  /// @param elevation_bounds elevation bounds of the lane, uniform along the
  ///        entire segment surface
  /// @param road_curve The trajectory of the Lane over parent @p segment's
  ///        surface.
  /// @param r0 The lateral displacement with respect to the @p road_curve's
  ///        reference curve.
  ///
  /// @note The override Lane::ToLanePosition() is currently restricted to
  /// lanes in which superelevation and elevation change are both zero.
  Lane(const api::LaneId& id, const api::Segment* segment, int index, const api::RBounds& lane_bounds,
       const api::RBounds& segment_bounds, const api::HBounds& elevation_bounds, const RoadCurve* road_curve, double r0)
      : id_(id),
        segment_(segment),
        index_(index),
        lane_bounds_(lane_bounds),
        segment_bounds_(segment_bounds),
        elevation_bounds_(elevation_bounds),
        road_curve_(road_curve),
        r0_(r0) {
    MALIPUT_DEMAND(index_ >= 0);
    MALIPUT_DEMAND(lane_bounds_.min() >= segment_bounds_.min());
    MALIPUT_DEMAND(lane_bounds_.max() <= segment_bounds_.max());
    MALIPUT_DEMAND(road_curve != nullptr);
    s_from_p_at_r0_ = road_curve_->OptimizeCalcSFromP(r0_);
    p_from_s_at_r0_ = road_curve_->OptimizeCalcPFromS(r0_);
    lane_length_ = s_from_p_at_r0_(1.0);
  }

  // TODO(maddog-tri)  Allow superelevation to have a center-of-rotation
  //                   which is different from r = 0.

  const CubicPolynomial& elevation() const { return road_curve_->elevation(); }

  const CubicPolynomial& superelevation() const { return road_curve_->superelevation(); }

  double r0() const { return r0_; }

  void SetStartBp(BranchPoint* bp) { start_bp_ = bp; }
  void SetEndBp(BranchPoint* bp) { end_bp_ = bp; }

  BranchPoint* start_bp() { return start_bp_; }

  BranchPoint* end_bp() { return end_bp_; }

  ~Lane() override = default;

 private:
  api::LaneId do_id() const override { return id_; }

  const api::Segment* do_segment() const override;

  int do_index() const override { return index_; }

  const api::Lane* do_to_left() const override {
    if (index_ == (segment_->num_lanes() - 1)) {
      return nullptr;
    } else {
      return segment_->lane(index_ + 1);
    }
  }

  const api::Lane* do_to_right() const override {
    if (index_ == 0) {
      return nullptr;
    } else {
      return segment_->lane(index_ - 1);
    }
  }

  const api::BranchPoint* DoGetBranchPoint(const api::LaneEnd::Which which_end) const override;

  const api::LaneEndSet* DoGetConfluentBranches(const api::LaneEnd::Which which_end) const override;

  const api::LaneEndSet* DoGetOngoingBranches(const api::LaneEnd::Which which_end) const override;

  std::optional<api::LaneEnd> DoGetDefaultBranch(const api::LaneEnd::Which which_end) const override;

  double do_length() const override { return lane_length_; }

  api::RBounds do_lane_bounds(double) const override { return lane_bounds_; }

  api::RBounds do_segment_bounds(double) const override { return segment_bounds_; }

  api::HBounds do_elevation_bounds(double, double) const override { return elevation_bounds_; }

  api::InertialPosition DoToInertialPosition(const api::LanePosition& lane_pos) const override;

  api::Rotation DoGetOrientation(const api::LanePosition& lane_pos) const override;

  api::LanePosition DoEvalMotionDerivatives(const api::LanePosition& position,
                                            const api::IsoLaneVelocity& velocity) const override;

  api::LanePositionResult DoToLanePosition(const api::InertialPosition& inertial_position) const override;

  api::LanePositionResult DoToSegmentPosition(const api::InertialPosition& inertial_position) const override;

  api::LanePositionResult InertialToLaneSegmentPositionBackend(const api::InertialPosition& inertial_position,
                                                               bool use_lane_boundaries) const;

  const api::LaneId id_;
  const api::Segment* segment_{};
  const int index_{};
  BranchPoint* start_bp_{};
  BranchPoint* end_bp_{};

  const api::RBounds lane_bounds_;
  const api::RBounds segment_bounds_;
  const api::HBounds elevation_bounds_;
  const RoadCurve* road_curve_{};
  const double r0_;

  std::function<double(double)> s_from_p_at_r0_;
  std::function<double(double)> p_from_s_at_r0_;
  double lane_length_;
};

}  // namespace multilane
}  // namespace maliput
