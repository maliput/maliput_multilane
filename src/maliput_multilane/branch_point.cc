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
#include "maliput_multilane/branch_point.h"

#include <maliput/common/maliput_abort.h>

namespace maliput {
namespace multilane {

BranchPoint::BranchPoint(const api::BranchPointId& id, const api::RoadGeometry* road_geometry)
    : id_(id), road_geometry_(road_geometry) {}

const api::RoadGeometry* BranchPoint::do_road_geometry() const { return road_geometry_; }

const api::LaneEndSet* BranchPoint::DoGetConfluentBranches(const api::LaneEnd& end) const {
  return confluent_branches_.at(end);
}

const api::LaneEndSet* BranchPoint::DoGetOngoingBranches(const api::LaneEnd& end) const {
  return ongoing_branches_.at(end);
}

std::optional<api::LaneEnd> BranchPoint::DoGetDefaultBranch(const api::LaneEnd& end) const {
  auto default_it = defaults_.find(end);
  if (default_it == defaults_.end()) {
    return std::nullopt;
  }
  return default_it->second;
}

const api::LaneEnd& BranchPoint::AddABranch(const api::LaneEnd& lane_end) {
  MALIPUT_DEMAND(confluent_branches_.find(lane_end) == confluent_branches_.end());
  MALIPUT_DEMAND(ongoing_branches_.find(lane_end) == ongoing_branches_.end());
  a_side_.add(lane_end);
  confluent_branches_[lane_end] = &a_side_;
  ongoing_branches_[lane_end] = &b_side_;
  return lane_end;
}

const api::LaneEnd& BranchPoint::AddBBranch(const api::LaneEnd& lane_end) {
  MALIPUT_DEMAND(confluent_branches_.find(lane_end) == confluent_branches_.end());
  MALIPUT_DEMAND(ongoing_branches_.find(lane_end) == ongoing_branches_.end());
  b_side_.add(lane_end);
  confluent_branches_[lane_end] = &b_side_;
  ongoing_branches_[lane_end] = &a_side_;
  return lane_end;
}

void BranchPoint::SetDefault(const api::LaneEnd& lane_end, const api::LaneEnd& default_branch) {
  const auto& le_ongoing = ongoing_branches_.find(lane_end);
  const auto& db_confluent = confluent_branches_.find(default_branch);
  // Verify that lane_end belongs to this BranchPoint.
  MALIPUT_DEMAND(le_ongoing != ongoing_branches_.end());
  // Verify that default_branch belongs to this BranchPoint.
  MALIPUT_DEMAND(db_confluent != confluent_branches_.end());
  // Verify that default_branch is an ongoing lane for lane_end.
  MALIPUT_DEMAND(db_confluent->second == le_ongoing->second);

  defaults_[lane_end] = default_branch;
}

}  // namespace multilane
}  // namespace maliput
