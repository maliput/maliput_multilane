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
/* clang-format off to disable clang-format-includes */
#include "maliput_multilane/segment.h"
/* clang-format on */

#include <gtest/gtest.h>
#include <maliput/api/compare.h>
#include <maliput/math/vector.h>

#include "assert_compare.h"
#include "maliput_multilane/arc_road_curve.h"
#include "maliput_multilane/junction.h"
#include "maliput_multilane/lane.h"
#include "maliput_multilane/line_road_curve.h"
#include "maliput_multilane/road_curve.h"
#include "maliput_multilane/road_geometry.h"

namespace maliput {
namespace multilane {
namespace {

using test::AssertCompare;

const double kLinearTolerance = 1e-6;
const double kAngularTolerance = 1e-6;
const double kZeroTolerance = 0.;

GTEST_TEST(MultilaneSegmentsTest, MultipleLanes) {
  CubicPolynomial zp{0., 0., 0., 0.};
  const double kR0 = 10.;
  const int kNumLanes = 3;
  const double kRSpacing = 15.;
  const double kRMin = 2.;
  const double kRMax = 42.;
  const double kHalfLaneWidth = 0.5 * kRSpacing;
  const double kMaxHeight = 5.;
  const double kScaleLength = 1.;
  const ComputationPolicy kComputationPolicy{ComputationPolicy::kPreferAccuracy};

  RoadGeometry rg(api::RoadGeometryId{"apple"}, kLinearTolerance, kAngularTolerance, kScaleLength);
  std::unique_ptr<RoadCurve> road_curve_1 = std::make_unique<LineRoadCurve>(
      math::Vector2(100., -75.), math::Vector2(100., 50.), zp, zp, kLinearTolerance, kScaleLength, kComputationPolicy);
  Segment* s1 = rg.NewJunction(api::JunctionId{"j1"})
                    ->NewSegment(api::SegmentId{"s1"}, std::move(road_curve_1), kRMin, kRMax, {0., kMaxHeight});
  EXPECT_EQ(s1->id(), api::SegmentId("s1"));
  EXPECT_EQ(s1->num_lanes(), 0);

  Lane* l0 = s1->NewLane(api::LaneId{"l0"}, kR0, {-8., kHalfLaneWidth});
  EXPECT_EQ(s1->num_lanes(), 1);
  EXPECT_EQ(s1->lane(0), l0);
  Lane* l1 = s1->NewLane(api::LaneId{"l1"}, kR0 + kRSpacing, {-kHalfLaneWidth, kHalfLaneWidth});
  EXPECT_EQ(s1->num_lanes(), 2);
  EXPECT_EQ(s1->lane(1), l1);
  Lane* l2 = s1->NewLane(api::LaneId{"l2"}, kR0 + 2. * kRSpacing, {-kHalfLaneWidth, 2.});
  EXPECT_EQ(s1->num_lanes(), kNumLanes);
  EXPECT_EQ(s1->lane(2), l2);

  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  EXPECT_TRUE(AssertCompare(api::IsRBoundsClose(l0->lane_bounds(0.), {-8., kHalfLaneWidth}, kZeroTolerance)));
  EXPECT_TRUE(AssertCompare(api::IsRBoundsClose(l0->segment_bounds(0.), {-8., 32.}, kZeroTolerance)));
  EXPECT_TRUE(
      AssertCompare(api::IsRBoundsClose(l1->lane_bounds(0.), {-kHalfLaneWidth, kHalfLaneWidth}, kZeroTolerance)));
  EXPECT_TRUE(AssertCompare(api::IsRBoundsClose(l1->segment_bounds(0.), {-23., 17.}, kZeroTolerance)));
  EXPECT_TRUE(AssertCompare(api::IsRBoundsClose(l2->lane_bounds(0.), {-kHalfLaneWidth, 2.}, kZeroTolerance)));
  EXPECT_TRUE(AssertCompare(api::IsRBoundsClose(l2->segment_bounds(0.), {-38., 2.}, kZeroTolerance)));
}

}  // namespace
}  // namespace multilane
}  // namespace maliput
