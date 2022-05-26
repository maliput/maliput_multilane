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
#include "maliput_multilane/line_road_curve.h"

#include <maliput/drake/math/saturate.h>

namespace maliput {
namespace multilane {

const double LineRoadCurve::kMinimumNorm = 1e-12;

double LineRoadCurve::FastCalcPFromS(double s, double r) const {
  maliput::common::unused(r);
  return elevation().p_s(s / l_max());
}

double LineRoadCurve::FastCalcSFromP(double p, double r) const {
  maliput::common::unused(r);
  return l_max() * elevation().s_p(p);
}

math::Vector3 LineRoadCurve::ToCurveFrame(const math::Vector3& geo_coordinate, double r_min, double r_max,
                                          const api::HBounds& height_bounds) const {
  MALIPUT_DEMAND(r_min <= r_max);
  // TODO(jadecastro): Lift the zero superelevation and zero elevation gradient
  // restriction.
  const math::Vector2 s_unit_vector = dp_ / dp_.norm();
  const math::Vector2 r_unit_vector{-s_unit_vector[1], s_unit_vector[0]};

  const math::Vector2 q(geo_coordinate.x(), geo_coordinate.y());
  const math::Vector2 lane_origin_to_q = q - p0_;

  // Compute the distance from `q` to the start of the lane.
  const double p_unsaturated = lane_origin_to_q.dot(s_unit_vector) / l_max();
  const double p = maliput::drake::math::saturate(p_unsaturated, 0., 1.);
  const double r_unsaturated = lane_origin_to_q.dot(r_unit_vector);
  const double r = maliput::drake::math::saturate(r_unsaturated, r_min, r_max);
  // N.B. h is the geo z-coordinate referenced against the lane elevation (whose
  // `a` coefficient is normalized by lane length).
  const double h_unsaturated = geo_coordinate.z() - elevation().a() * l_max();
  const double h = maliput::drake::math::saturate(h_unsaturated, height_bounds.min(), height_bounds.max());
  return math::Vector3(p, r, h);
}

}  // namespace multilane
}  // namespace maliput
