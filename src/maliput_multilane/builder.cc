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
#include "maliput_multilane/builder.h"

#include <cmath>
#include <utility>

#include <maliput/common/logger.h>
#include <maliput/common/maliput_abort.h>

#include "maliput_multilane/arc_road_curve.h"
#include "maliput_multilane/branch_point.h"
#include "maliput_multilane/cubic_polynomial.h"
#include "maliput_multilane/junction.h"
#include "maliput_multilane/line_road_curve.h"
#include "maliput_multilane/make_road_curve_for_connection.h"
#include "maliput_multilane/road_geometry.h"

namespace maliput {
namespace multilane {

std::ostream& operator<<(std::ostream& out, const StartReference::Spec& start_spec) {
  return out << "(endpoint: " << start_spec.endpoint() << ")";
}

std::ostream& operator<<(std::ostream& out, const StartLane::Spec& start_spec) {
  return out << "(endpoint: " << start_spec.endpoint() << ", lane_id: " << start_spec.lane_id() << ")";
}

std::ostream& operator<<(std::ostream& out, const EndReference::Spec& end_spec) {
  return out << "(endpoint_z: " << end_spec.endpoint_z() << ")";
}

std::ostream& operator<<(std::ostream& out, const EndLane::Spec& end_spec) {
  return out << "(endpoint_z: " << end_spec.endpoint_z() << ", lane_id: " << end_spec.lane_id() << ")";
}

std::ostream& operator<<(std::ostream& out, const LaneLayout& lane_layout) {
  return out << "(left_shoulder: " << lane_layout.left_shoulder()
             << ", right_shoulder: " << lane_layout.right_shoulder() << ", num_lanes: " << lane_layout.num_lanes()
             << ", ref_lane: " << lane_layout.ref_lane() << ", ref_r0: " << lane_layout.ref_r0() << ")";
}

Builder::Builder(double lane_width, const api::HBounds& elevation_bounds, double linear_tolerance,
                 double angular_tolerance, double scale_length, ComputationPolicy computation_policy,
                 std::unique_ptr<GroupFactoryBase> group_factory)
    : lane_width_(lane_width),
      elevation_bounds_(elevation_bounds),
      linear_tolerance_(linear_tolerance),
      angular_tolerance_(angular_tolerance),
      scale_length_(scale_length),
      computation_policy_(computation_policy),
      group_factory_(std::move(group_factory)) {
  MALIPUT_DEMAND(lane_width_ >= 0.);
  MALIPUT_DEMAND(linear_tolerance_ >= 0.);
  MALIPUT_DEMAND(angular_tolerance_ >= 0.);
  MALIPUT_DEMAND(group_factory_ != nullptr);
}

namespace {
// Let theta_dot_A, z_dot_A, K_A be the rate of change in superelevation with
// respect to arc length of the reference path, grade (rate of change of
// elevation with respect to arc length of the reference path) and the
// curvature at one end point of connection's curve A. And let theta_dot_B,
// z_dot_B, K_B be the same values at one end point of connection's curve
// B. Then, if we want to connect both curves at their respective end points A
// and B, making:
//
// theta_dot_A - (K_A * sin(-atan(z_dot_A))) = 0
// theta_dot_B - (K_B * sin(-atan(z_dot_B))) = 0
//
// It is enough to make the connection joint be G1. Given that, `curvature` is
// K and `endpointz` is z_dot. `endpointz.theta_dot` is computed as:
//
// theta_dot = K * sin(-atan(z_dot))
void ComputeContinuityConstraint(double curvature, EndpointZ* endpointz) {
  MALIPUT_DEMAND(endpointz != nullptr);
  endpointz->get_mutable_theta_dot() = curvature * std::sin(-std::atan(endpointz->z_dot()));
}

// EndpointFuzzyOrder is an arbitrary strict complete ordering of Endpoints
// useful for, e.g., std::map.  It provides a comparison operation that
// treats two Endpoints within `linear_tolerance` of one another as
// equivalent.
//
// This is used to match up the endpoints of Connections, to determine
// how Connections are linked to one another.  Exact numeric equality
// would not be robust given the use of floating-point values in Endpoints.
class EndpointFuzzyOrder {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EndpointFuzzyOrder)

  // clang-format off
  explicit EndpointFuzzyOrder(const double linear_tolerance)
      : lin_tol_(linear_tolerance) {}
  // clang-format on

  bool operator()(const Endpoint& lhs, const Endpoint& rhs) const {
    switch (fuzzy_compare(rhs.xy().x(), lhs.xy().x())) {
      default:
        break;
      case -1: {
        return true;
      }
      case 1: {
        return false;
      }
      case 0: {
        switch (fuzzy_compare(rhs.xy().y(), lhs.xy().y())) {
          default:
            break;
          case -1: {
            return true;
          }
          case 1: {
            return false;
          }
          case 0: {
            switch (fuzzy_compare(rhs.z().z(), lhs.z().z())) {
              default:
                break;
              case -1: {
                return true;
              }
              case 1: {
                return false;
              }
              case 0: {
                return false;
              }
            }
          }
        }
      }
    }
    throw std::domain_error("fuzzy_compare domain error");
  }

 private:
  int fuzzy_compare(const double a, const double b) const {
    if (a < (b - lin_tol_)) {
      return -1;
    } else if (a > (b + lin_tol_)) {
      return 1;
    } else {
      return 0;
    }
  }

  double lin_tol_{};
};

}  // namespace

const Connection* Builder::Connect(const std::string& id, const LaneLayout& lane_layout,
                                   const StartReference::Spec& start_spec, const LineOffset& line_offset,
                                   const EndReference::Spec& end_spec) {
  // TODO(agalbachicar)    Once the API supports referencing to lanes, this
  //                       should be used to call the appropriate Builder
  //                       methods, r_ref will refer to any lane and will not be
  //                       r0.
  MALIPUT_DEMAND(lane_layout.ref_lane() == 0);

  const double curvature{0.};
  Endpoint start_endpoint = start_spec.endpoint();
  if (!start_endpoint.z().theta_dot()) {
    ComputeContinuityConstraint(curvature, &(start_endpoint.get_mutable_z()));
  }
  EndpointZ end_endpoint_z = end_spec.endpoint_z();
  if (!end_endpoint_z.theta_dot()) {
    ComputeContinuityConstraint(curvature, &end_endpoint_z);
  }
  connections_.push_back(std::make_unique<Connection>(id, start_endpoint, end_endpoint_z, lane_layout.num_lanes(),
                                                      lane_layout.ref_r0(), lane_width_, lane_layout.left_shoulder(),
                                                      lane_layout.right_shoulder(), line_offset, linear_tolerance_,
                                                      scale_length_, computation_policy_));
  return connections_.back().get();
}

const Connection* Builder::Connect(const std::string& id, const LaneLayout& lane_layout,
                                   const StartReference::Spec& start_spec, const ArcOffset& arc_offset,
                                   const EndReference::Spec& end_spec) {
  // TODO(agalbachicar)    Once the API supports referencing to lanes, this
  //                       should be used to call the appropriate Builder
  //                       methods, r_ref will refer to any lane and will not be
  //                       r0.
  MALIPUT_DEMAND(lane_layout.ref_lane() == 0);

  const double curvature = std::copysign(1., arc_offset.d_theta()) / arc_offset.radius();

  Endpoint start_endpoint = start_spec.endpoint();
  if (!start_endpoint.z().theta_dot()) {
    ComputeContinuityConstraint(curvature, &(start_endpoint.get_mutable_z()));
  }
  EndpointZ end_endpoint_z = end_spec.endpoint_z();
  if (!end_endpoint_z.theta_dot()) {
    ComputeContinuityConstraint(curvature, &end_endpoint_z);
  }
  connections_.push_back(std::make_unique<Connection>(id, start_endpoint, end_endpoint_z, lane_layout.num_lanes(),
                                                      lane_layout.ref_r0(), lane_width_, lane_layout.left_shoulder(),
                                                      lane_layout.right_shoulder(), arc_offset, linear_tolerance_,
                                                      scale_length_, computation_policy_));
  return connections_.back().get();
}

// TODO(hidmic): Support connections where the constant C in the
//               continuity constraints theta_dotA - sin(betaA) = C
//               and theta_dotB - sin(betaB) = C is non-zero. This
//               is necessary to ensure continuity when arbitrary
//               theta_dot values are provided by the user. Then,
//               do not clear *explicitly* set theta_dot values
//               upon Spec construction using a Connection anymore.

const Connection* Builder::Connect(const std::string& id, const LaneLayout& lane_layout,
                                   const StartLane::Spec& start_spec, const LineOffset& line_offset,
                                   const EndLane::Spec& end_spec) {
  MALIPUT_DEMAND(start_spec.lane_id() >= 0 && start_spec.lane_id() < lane_layout.num_lanes());
  MALIPUT_DEMAND(end_spec.lane_id() >= 0 && end_spec.lane_id() < lane_layout.num_lanes());

  const double curvature{0.};

  // Gets the initial heading and superelevation.
  const double start_heading = start_spec.endpoint().xy().heading();
  const double start_superelevation = start_spec.endpoint().z().theta();
  const math::Vector3 start_lane_position(
      {start_spec.endpoint().xy().x(), start_spec.endpoint().xy().y(), start_spec.endpoint().z().z()});
  const double start_lane_offset =
      -lane_layout.ref_r0() + lane_width_ * static_cast<double>(start_spec.lane_id() - lane_layout.ref_lane());

  // Gets the rotation matrix at the start lane endpoint and translates the
  // point to start_reference_position.
  const api::Rotation start_rotation =
      api::Rotation::FromRpy(start_superelevation, -std::atan(start_spec.endpoint().z().z_dot()), start_heading);
  const math::Vector3 start_reference_position{start_lane_position +
                                               start_rotation.matrix() * math::Vector3(0., -start_lane_offset, 0.)};

  // Assigns the start endpoint.
  Endpoint start{EndpointXy(start_reference_position.x(), start_reference_position.y(), start_heading),
                 EndpointZ(start_reference_position.z(), start_spec.endpoint().z().z_dot(),
                           start_spec.endpoint().z().theta(), {})};
  ComputeContinuityConstraint(curvature, &(start.get_mutable_z()));

  // Computes the r distance from the end lane to the reference curve.
  const double end_lane_offset =
      -lane_layout.ref_r0() + lane_width_ * static_cast<double>(end_spec.lane_id() - lane_layout.ref_lane());
  // Computes the end endpoint.
  const double z_end = end_spec.endpoint_z().z() - end_lane_offset * std::sin(end_spec.endpoint_z().theta());
  EndpointZ end_z{z_end, end_spec.endpoint_z().z_dot(), end_spec.endpoint_z().theta(), {}};
  ComputeContinuityConstraint(curvature, &end_z);

  connections_.push_back(
      std::make_unique<Connection>(id, start, end_z, lane_layout.num_lanes(),
                                   -lane_layout.ref_r0() - static_cast<double>(lane_layout.ref_lane()) * lane_width_,
                                   lane_width_, lane_layout.left_shoulder(), lane_layout.right_shoulder(), line_offset,
                                   linear_tolerance_, scale_length_, computation_policy_));
  return connections_.back().get();
}

const Connection* Builder::Connect(const std::string& id, const LaneLayout& lane_layout,
                                   const StartLane::Spec& start_spec, const ArcOffset& arc_offset,
                                   const EndLane::Spec& end_spec) {
  MALIPUT_DEMAND(start_spec.lane_id() >= 0 && start_spec.lane_id() < lane_layout.num_lanes());
  MALIPUT_DEMAND(end_spec.lane_id() >= 0 && end_spec.lane_id() < lane_layout.num_lanes());

  const double curvature = std::copysign(1., arc_offset.d_theta()) / arc_offset.radius();

  // Gets the initial heading and superelevation.
  const double start_superelevation = start_spec.endpoint().z().theta();

  // Computes the start_lane_radius to scale z_dot.
  const double start_lane_offset =
      -lane_layout.ref_r0() + lane_width_ * static_cast<double>(start_spec.lane_id() - lane_layout.ref_lane());
  const double start_lane_radius = arc_offset.radius() - start_lane_offset * std::cos(start_superelevation) *
                                                             std::copysign(1., arc_offset.d_theta());
  const double start_z_dot = start_spec.endpoint().z().z_dot() * start_lane_radius / arc_offset.radius();

  // Gets the rotation matrix at the start lane endpoint and translates the
  // point to start_reference_position.
  const api::Rotation start_rotation =
      api::Rotation::FromRpy(start_superelevation, -std::atan(start_z_dot), start_spec.endpoint().xy().heading());
  const math::Vector3 start_lane_position{start_spec.endpoint().xy().x(), start_spec.endpoint().xy().y(),
                                          start_spec.endpoint().z().z()};
  const math::Vector3 start_reference_position{start_lane_position +
                                               start_rotation.matrix() * math::Vector3(0., -start_lane_offset, 0.)};

  // Assigns the start endpoint.
  Endpoint start{
      EndpointXy(start_reference_position.x(), start_reference_position.y(), start_spec.endpoint().xy().heading()),
      EndpointZ(start_reference_position.z(), start_z_dot, start_spec.endpoint().z().theta(), {})};
  ComputeContinuityConstraint(curvature, &(start.get_mutable_z()));

  // Assigns the end endpoint.
  const double end_superelevation = end_spec.endpoint_z().theta();
  const double end_lane_offset =
      -lane_layout.ref_r0() + lane_width_ * static_cast<double>(end_spec.lane_id() - lane_layout.ref_lane());
  const double end_z_dot =
      end_spec.endpoint_z().z_dot() *
      (arc_offset.radius() - end_lane_offset * std::cos(end_superelevation) * std::copysign(1., arc_offset.d_theta())) /
      arc_offset.radius();
  EndpointZ end_z{end_spec.endpoint_z().z() -
                      end_lane_offset * std::sin(end_superelevation) * std::copysign(1., arc_offset.d_theta()),
                  end_z_dot,
                  end_spec.endpoint_z().theta(),
                  {}};
  ComputeContinuityConstraint(curvature, &end_z);

  connections_.push_back(
      std::make_unique<Connection>(id, start, end_z, lane_layout.num_lanes(),
                                   -lane_layout.ref_r0() - static_cast<double>(lane_layout.ref_lane()) * lane_width_,
                                   lane_width_, lane_layout.left_shoulder(), lane_layout.right_shoulder(), arc_offset,
                                   linear_tolerance_, scale_length_, computation_policy_));
  return connections_.back().get();
}

void Builder::SetDefaultBranch(const Connection* in, int in_lane_index, const api::LaneEnd::Which in_end,
                               const Connection* out, int out_lane_index, const api::LaneEnd::Which out_end) {
  default_branches_.push_back({in, in_lane_index, in_end, out, out_lane_index, out_end});
}

Group* Builder::MakeGroup(const std::string& id) {
  groups_.push_back(group_factory_->Make(id));
  return groups_.back().get();
}

Group* Builder::MakeGroup(const std::string& id, const std::vector<const Connection*>& connections) {
  groups_.push_back(group_factory_->Make(id, connections));
  return groups_.back().get();
}

namespace {

// Determine the heading direction at an `r_offset` leftwards of the `lane`
// centerline when traveling outwards from the specified `end`.
math::Vector3 DirectionOutFromLane(const api::Lane* const lane, const api::LaneEnd::Which end, double r_offset) {
  const math::Vector3 s_hat = math::Vector3::UnitX();

  switch (end) {
    case api::LaneEnd::kStart: {
      return -1 * (lane->GetOrientation({0., -r_offset, 0.}).matrix() * s_hat);
    }
    case api::LaneEnd::kFinish: {
      return lane->GetOrientation({lane->length(), r_offset, 0.}).matrix() * s_hat;
    }
  }
  MALIPUT_ABORT_MESSAGE("end is neither LaneEnd::kStart nor LaneEnd::KFinish.");
}

// Checks if the heading direction of the all the given lanes at an `r_offset`
// leftwards of the centerlines when traveling outwards from their specified
// end in `set` matches the reference `direction_at_r_offset`, down to the
// given `angular_tolerance`.
bool AreLanesDirectionsWithinTolerance(const math::Vector3& direction_at_r_offset, double r_offset,
                                       const api::LaneEndSet* set, double angular_tolerance) {
  MALIPUT_DEMAND(set != nullptr);

  for (int i = 0; i < set->size(); ++i) {
    const api::LaneEnd& le = set->get(i);
    const math::Vector3 other_direction = DirectionOutFromLane(le.lane, le.end, r_offset);
    const double angular_deviation = std::acos(direction_at_r_offset.dot(other_direction));
    if (angular_deviation > angular_tolerance) {
      return false;
    }
  }
  return true;
}

// Checks if the `lane` is G1 continuous along an `r_offset` leftwards of
// the `lane` centerline when traveling outwards from the given `end`, at the
// branch point and down to the given `angular_tolerance`. `lane` is assumed
// to have an out-of-the-lane tangent vector that is coincident with those of
// the lanes in the `parallel_set`, which in turn are anti-parallel to
// those of the lanes in the `antiparallel_set`.
bool IsLaneContinuousAtBranchPointAlongROffset(const api::Lane* lane, const api::LaneEnd::Which end,
                                               const api::LaneEndSet* parallel_set,
                                               const api::LaneEndSet* antiparallel_set, double r_offset,
                                               double angular_tolerance) {
  MALIPUT_DEMAND(lane != nullptr);
  MALIPUT_DEMAND(parallel_set != nullptr);
  MALIPUT_DEMAND(antiparallel_set != nullptr);

  const math::Vector3 direction = DirectionOutFromLane(lane, end, r_offset);
  return (AreLanesDirectionsWithinTolerance(direction, r_offset, parallel_set, angular_tolerance) &&
          AreLanesDirectionsWithinTolerance(-1 * direction, r_offset, antiparallel_set, angular_tolerance));
}

// Checks if the `lane` is G1 continuous at the branchpoint on the given
// `end`, down to an `angular_tolerance`. `lane` is assumed to have an
// out-of-the-lane tangent vector that is coincident with those of the
// lanes in the `parallel_set`, which in turn are anti-parallel to those
// of the lanes in the `antiparallel_set`.
bool IsLaneContinuousAtBranchPoint(const api::Lane* lane, const api::LaneEnd::Which end,
                                   const api::LaneEndSet* parallel_set, const api::LaneEndSet* antiparallel_set,
                                   double angular_tolerance) {
  MALIPUT_DEMAND(lane != nullptr);
  MALIPUT_DEMAND(parallel_set != nullptr);
  MALIPUT_DEMAND(antiparallel_set != nullptr);

  constexpr double kZeroROffset{0.};
  const double s_at_branch_point = (end == api::LaneEnd::kFinish) ? lane->length() : 0.;
  const double max_r_offset = lane->lane_bounds(s_at_branch_point).max();
  return (IsLaneContinuousAtBranchPointAlongROffset(lane, end, parallel_set, antiparallel_set, kZeroROffset,
                                                    angular_tolerance) &&
          IsLaneContinuousAtBranchPointAlongROffset(lane, end, parallel_set, antiparallel_set, max_r_offset,
                                                    angular_tolerance));
}

BranchPoint* FindOrCreateBranchPoint(const Endpoint& point, RoadGeometry* road_geometry,
                                     std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder>* bp_map) {
  auto ibp = bp_map->find(point);
  if (ibp != bp_map->end()) {
    return ibp->second;
  }
  // TODO(maddog@tri.global) Generate a more meaningful id (user-specified?)
  BranchPoint* bp =
      road_geometry->NewBranchPoint(api::BranchPointId{"bp:" + std::to_string(road_geometry->num_branch_points())});
  auto result = bp_map->emplace(point, bp);
  MALIPUT_DEMAND(result.second);
  return bp;
}

void AttachBranchPoint(const Endpoint& point, Lane* const lane, const api::LaneEnd::Which end, double angular_tolerance,
                       RoadGeometry* road_geometry, std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder>* bp_map) {
  BranchPoint* bp = FindOrCreateBranchPoint(point, road_geometry, bp_map);
  // Tell the lane about its branch-point.
  MALIPUT_DEMAND((end == api::LaneEnd::kStart) || (end == api::LaneEnd::kFinish));
  switch (end) {
    case api::LaneEnd::kStart: {
      lane->SetStartBp(bp);
      break;
    }
    case api::LaneEnd::kFinish: {
      lane->SetEndBp(bp);
      break;
    }
  }
  // Now, tell the branch-point about the lane.
  //
  // Is this the first lane-end added to the branch-point?
  // If so, just stick it on A-Side.
  // (NB: We just test size of A-Side, since A-Side is always populated first.)
  if (bp->GetASide()->size() == 0) {
    bp->AddABranch({lane, end});
    return;
  }

  // Otherwise, assess if this new lane-end is parallel or anti-parallel to
  // the first lane-end.  Parallel: go to same, A-side; anti-parallel:
  // other, B-side.  Do this by examining the dot-product of the heading
  // vectors (rather than goofing around with cyclic angle arithmetic).
  constexpr double kZeroROffset{0.};
  const math::Vector3 direction = DirectionOutFromLane(lane, end, kZeroROffset);
  const api::LaneEnd& old_le = bp->GetASide()->get(0);
  const math::Vector3 old_direction = DirectionOutFromLane(old_le.lane, old_le.end, kZeroROffset);
  if (direction.dot(old_direction) > 0.) {
    // Assert continuity before attaching the lane.
    MALIPUT_THROW_UNLESS(IsLaneContinuousAtBranchPoint(lane, end, bp->GetASide(), bp->GetBSide(), angular_tolerance));
    bp->AddABranch({lane, end});
  } else {
    // Assert continuity before attaching the lane.
    MALIPUT_THROW_UNLESS(IsLaneContinuousAtBranchPoint(lane, end, bp->GetBSide(), bp->GetASide(), angular_tolerance));
    bp->AddBBranch({lane, end});
  }
}

std::vector<Lane*> BuildConnection(const Connection* const conn, Junction* const junction,
                                   const api::HBounds& elevation_bounds, double angular_tolerance,
                                   RoadGeometry* const road_geometry,
                                   std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder>* const bp_map) {
  Segment* segment = junction->NewSegment(api::SegmentId{std::string("s:") + conn->id()}, MakeRoadCurveFor(*conn),
                                          conn->r_min(), conn->r_max(), elevation_bounds);
  std::vector<Lane*> lanes;
  for (int i = 0; i < conn->num_lanes(); i++) {
    Lane* lane = segment->NewLane(api::LaneId{std::string("l:") + conn->id() + std::string("_") + std::to_string(i)},
                                  conn->lane_offset(i), {-conn->lane_width() / 2., conn->lane_width() / 2.});
    AttachBranchPoint(conn->LaneStart(i), lane, api::LaneEnd::kStart, angular_tolerance, road_geometry, bp_map);
    AttachBranchPoint(conn->LaneEnd(i), lane, api::LaneEnd::kFinish, angular_tolerance, road_geometry, bp_map);
    lanes.push_back(lane);
  }
  return lanes;
}

}  // namespace

std::unique_ptr<const api::RoadGeometry> Builder::Build(const api::RoadGeometryId& id) const {
  auto road_geometry = std::make_unique<RoadGeometry>(id, linear_tolerance_, angular_tolerance_, scale_length_);
  std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder> bp_map((EndpointFuzzyOrder(linear_tolerance_)));
  std::map<const Connection*, std::vector<Lane*>> lane_map;
  std::map<const Connection*, bool> connection_was_built;

  for (const std::unique_ptr<Connection>& connection : connections_) {
    connection_was_built.emplace(connection.get(), false);
  }

  for (const std::unique_ptr<Group>& group : groups_) {
    Junction* junction = road_geometry->NewJunction(api::JunctionId{std::string("j:") + group->id()});
    maliput::log()->debug("junction: ", junction->id().string());
    for (auto& connection : group->connections()) {
      maliput::log()->debug("connection: ", connection->id());
      MALIPUT_DEMAND(!connection_was_built[connection]);
      lane_map[connection] =
          BuildConnection(connection, junction, elevation_bounds_, angular_tolerance_, road_geometry.get(), &bp_map);
      connection_was_built[connection] = true;
    }
  }

  for (const std::unique_ptr<Connection>& connection : connections_) {
    if (connection_was_built[connection.get()]) {
      continue;
    }
    Junction* junction = road_geometry->NewJunction(api::JunctionId{std::string("j:") + connection->id()});
    maliput::log()->debug("junction: ", junction->id().string());
    maliput::log()->debug("connection: ", connection->id());
    lane_map[connection.get()] = BuildConnection(connection.get(), junction, elevation_bounds_, angular_tolerance_,
                                                 road_geometry.get(), &bp_map);
  }

  for (const DefaultBranch& def : default_branches_) {
    Lane* in_lane = lane_map[def.in][def.in_lane_index];
    Lane* out_lane = lane_map[def.out][def.out_lane_index];
    MALIPUT_DEMAND((def.in_end == api::LaneEnd::kStart) || (def.in_end == api::LaneEnd::kFinish));
    ((def.in_end == api::LaneEnd::kStart) ? in_lane->start_bp() : in_lane->end_bp())
        ->SetDefault({in_lane, def.in_end}, {out_lane, def.out_end});
  }

  // Make sure we didn't screw up!
  std::vector<std::string> failures = road_geometry->CheckInvariants();
  for (const auto& s : failures) {
    maliput::log()->error(s);
  }
  MALIPUT_DEMAND(failures.size() == 0);

  return std::move(road_geometry);
}

}  // namespace multilane
}  // namespace maliput
