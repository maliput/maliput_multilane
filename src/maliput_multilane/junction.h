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
#include <vector>

#include <maliput/api/junction.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/segment.h>
#include <maliput/common/maliput_copyable.h>

#include "maliput_multilane/road_curve.h"
#include "maliput_multilane/segment.h"

namespace maliput {
namespace multilane {

/// An api::Junction implementation.
class Junction : public api::Junction {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Junction);

  /// Constructs an empty Junction.
  ///
  /// @p road_geometry must remain valid for the lifetime of this class,
  /// and must refer to the RoadGeometry which will contain the newly
  /// constructed Junction instance.
  /// @p register_segment and @p register_lane will be called on any new
  /// Segment or Lane instances created as children of the Junction.
  Junction(const api::JunctionId& id, const api::RoadGeometry* road_geometry,
           const std::function<void(const api::Segment*)>& register_segment,
           const std::function<void(const api::Lane*)>& register_lane)
      : id_(id), road_geometry_(road_geometry), register_segment_(register_segment), register_lane_(register_lane) {}

  /// Creates and adds a new Segment.
  /// @param id Segment's ID.
  /// @param road_curve Reference trajectory over the Segment's surface.
  /// @param r_min Lateral distance to the minimum extent of road_curve's curve
  /// from where Segment's surface starts.
  /// @param r_max Lateral distance to the maximum extent of road_curve's curve
  /// from where Segment's surface ends.
  /// @param elevation_bounds The height bounds over the segment' surface.
  /// @return A Segment object.
  Segment* NewSegment(const api::SegmentId& id, std::unique_ptr<RoadCurve> road_curve, double r_min, double r_max,
                      const api::HBounds& elevation_bounds);

  ~Junction() override = default;

 private:
  api::JunctionId do_id() const override { return id_; }

  const api::RoadGeometry* do_road_geometry() const override;

  int do_num_segments() const override { return segments_.size(); }

  const api::Segment* do_segment(int index) const override { return segments_[index].get(); }

  api::JunctionId id_;
  const api::RoadGeometry* road_geometry_{};
  std::function<void(const api::Segment*)> register_segment_;
  std::function<void(const api::Lane*)> register_lane_;
  std::vector<std::unique_ptr<Segment>> segments_;
};

}  // namespace multilane
}  // namespace maliput
