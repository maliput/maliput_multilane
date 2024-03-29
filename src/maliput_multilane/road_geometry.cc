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
#include "maliput_multilane/road_geometry.h"

#include <maliput/api/junction.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/segment.h>
#include <maliput/common/maliput_abort.h>
#include <maliput/common/maliput_unused.h>
#include <maliput/geometry_base/brute_force_find_road_positions_strategy.h>

namespace maliput {
namespace multilane {

namespace {

// Converts `result` and `lane` into an api::RoadPositionResult.
// `result.nearest_position` and `result.distance` are direct mappings, and the
// api::RoadPosition is built with `lane` and `result.lane_position`.
api::RoadPositionResult FromLanePositionResult(const api::Lane* const lane, const api::LanePositionResult& result) {
  return {api::RoadPosition{lane, result.lane_position}, result.nearest_position, result.distance};
}

// Evaluates if the result of `lane->ToSegmentPosition()` using `inertial_position`
// provides a closer api::RoadPositionResult than `road_position_result`.
//
// _Closer_ means:
// - When the distance result of `lane->ToSegmentPosition()` is smaller than
//   `road_position_result.distance` by more than `linear_tolerance`.
// - When distances are equal, if both positions fall within their
//   respective lane's lane bounds or none do, the new r-coordinate
//   is smaller than `road_position_result.road_position.pos.r()`.
// - When the new position falls within `lane`'s lane bounds and
//   `road_position_result.road_position.pos` doesn't.
//
// When any of the previous conditions is met, an api::RoadPositionResult
// is returned using `lane`, and the results of calling `ToSegmentPosition()`
// on it. Otherwise, it returns the original `road_position_result`.
//
// The following preconditions should be met:
//
// `lane` must not be nullptr.
const api::RoadPositionResult EvaluateRoadPositionResult(const api::InertialPosition& inertial_position,
                                                         const double linear_tolerance, const api::Lane* const lane,
                                                         const api::RoadPositionResult& road_position_result) {
  MALIPUT_DEMAND(lane != nullptr);

  const api::RoadPositionResult new_road_position_result =
      FromLanePositionResult(lane, lane->ToSegmentPosition(inertial_position));

  const double delta = new_road_position_result.distance - road_position_result.distance;
  if (delta > linear_tolerance) {  // new_distance is bigger than *distance, so this LanePosition is discarded.
    return road_position_result;
  }
  if (delta < -linear_tolerance) {  // It is a better match.
    return new_road_position_result;
  }

  auto is_within_lane_bounds = [](double r, const api::RBounds& lane_bounds) {
    return r >= lane_bounds.min() && r < lane_bounds.max();
  };
  // They are almost equal so it is worth checking the `r` coordinate and the
  // lane bounds.
  // When both r-coordinates fall within lane bounds or outside, the position
  // with the minimum absolute r-coordinate prevails.
  // When the new r-coordinate is within lane bounds, and the previous position
  // does not fall within lane bounds, the new result prevails.
  const api::RBounds new_lane_bounds = lane->lane_bounds(new_road_position_result.road_position.pos.s());
  const api::RBounds current_lane_bounds =
      road_position_result.road_position.lane->lane_bounds(road_position_result.road_position.pos.s());
  const bool is_new_within_lane_bounds =
      is_within_lane_bounds(new_road_position_result.road_position.pos.r(), new_lane_bounds);
  const bool is_current_within_lane_bounds =
      is_within_lane_bounds(road_position_result.road_position.pos.r(), current_lane_bounds);
  if ((is_new_within_lane_bounds && is_current_within_lane_bounds) ||
      (!is_new_within_lane_bounds && !is_current_within_lane_bounds)) {
    if (std::abs(new_road_position_result.road_position.pos.r()) <
        std::abs(road_position_result.road_position.pos.r())) {
      return new_road_position_result;
    }
  } else if (is_new_within_lane_bounds && !is_current_within_lane_bounds) {
    return new_road_position_result;
  }
  return road_position_result;
}

}  // namespace

Junction* RoadGeometry::NewJunction(api::JunctionId id) {
  namespace sp = std::placeholders;
  // clang-format off
  junctions_.push_back(std::make_unique<Junction>(id, this, [this](auto segment) { id_index_.AddSegment(segment); },
                                                  [this](auto lane) { id_index_.AddLane(lane); }));
  // clang-format on
  Junction* junction = junctions_.back().get();
  id_index_.AddJunction(junction);
  return junction;
}

BranchPoint* RoadGeometry::NewBranchPoint(api::BranchPointId id) {
  branch_points_.push_back(std::make_unique<BranchPoint>(id, this));
  BranchPoint* branch_point = branch_points_.back().get();
  id_index_.AddBranchPoint(branch_point);
  return branch_point;
}

const api::Junction* RoadGeometry::do_junction(int index) const { return junctions_[index].get(); }

const api::BranchPoint* RoadGeometry::do_branch_point(int index) const { return branch_points_[index].get(); }

api::RoadPositionResult RoadGeometry::DoToRoadPosition(const api::InertialPosition& inertial_position,
                                                       const std::optional<api::RoadPosition>& hint) const {
  api::RoadPositionResult road_position_result;

  // If a `hint` is supplied, simply use the `hint` lane; otherwise, search
  // through the any adjacent ongoing lanes for positions with smaller
  // distances.
  //
  // Note that this can be made more robust by extending this with a search that
  // extends beyond only adjacent lanes.
  if (hint.has_value()) {
    MALIPUT_DEMAND(hint->lane != nullptr);
    road_position_result = FromLanePositionResult(hint->lane, hint->lane->ToSegmentPosition(inertial_position));
    if (road_position_result.distance != 0.) {
      // Loop through ongoing lanes at both ends of the current lane, to find
      // the position associated with the first found containing lane or the
      // distance-minimizing position.
      for (const auto which_end : {api::LaneEnd::kStart, api::LaneEnd::kFinish}) {
        const api::LaneEndSet* ends = hint->lane->GetOngoingBranches(which_end);
        for (int i = 0; i < ends->size(); ++i) {
          road_position_result =
              EvaluateRoadPositionResult(inertial_position, linear_tolerance_, ends->get(i).lane, road_position_result);
        }
      }
    }

  } else {
    // No `hint` supplied.  Search exhaustively through all of the lanes to find
    // the position associated with the first found containing lane or the
    // distance-minimizing position.
    MALIPUT_DEMAND(num_junctions() > 0);
    MALIPUT_DEMAND(junction(0)->num_segments() > 0);
    MALIPUT_DEMAND(junction(0)->segment(0)->num_lanes() > 0);
    const api::Lane* lane = this->junction(0)->segment(0)->lane(0);
    road_position_result = FromLanePositionResult(lane, lane->ToSegmentPosition(inertial_position));
    for (int i = 0; i < num_junctions(); ++i) {
      const api::Junction* junction = this->junction(i);
      for (int j = 0; j < junction->num_segments(); ++j) {
        const api::Segment* segment = junction->segment(j);
        for (int k = 0; k < segment->num_lanes(); ++k) {
          road_position_result =
              EvaluateRoadPositionResult(inertial_position, linear_tolerance_, segment->lane(k), road_position_result);
        }
      }
    }
  }

  return road_position_result;
}

std::vector<api::RoadPositionResult> RoadGeometry::DoFindRoadPositions(const api::InertialPosition& inertial_position,
                                                                       double radius) const {
  return maliput::geometry_base::BruteForceFindRoadPositionsStrategy(this, inertial_position, radius);
}

}  // namespace multilane
}  // namespace maliput
