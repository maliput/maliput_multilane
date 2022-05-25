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
#include "maliput_multilane/segment.h"

namespace maliput {
namespace multilane {

const api::Junction* Segment::do_junction() const { return junction_; }

Lane* Segment::NewLane(api::LaneId id, double r0, const api::RBounds& lane_bounds) {
  const int index = lanes_.size();
  MALIPUT_DEMAND(r_min_ <= r0 && r0 <= r_max_);
  if (lanes_.size() != 0) {
    MALIPUT_DEMAND(r0 > lanes_.back()->r0());
  }
  const api::RBounds segment_bounds(r_min_ - r0, r_max_ - r0);
  MALIPUT_DEMAND(lane_bounds.min() >= segment_bounds.min() && lane_bounds.max() <= segment_bounds.max());
  auto lane_ =
      std::make_unique<Lane>(id, this, index, lane_bounds, segment_bounds, elevation_bounds_, road_curve_.get(), r0);
  lanes_.push_back(std::move(lane_));
  Lane* result = lanes_.back().get();
  register_lane_(result);
  return result;
}

const api::Lane* Segment::do_lane(int index) const {
  MALIPUT_DEMAND(index >= 0 && index < static_cast<int>(lanes_.size()));
  return lanes_[index].get();
}

}  // namespace multilane
}  // namespace maliput
