#include "maliput_multilane/connection.h"

#include <maliput/math/vector.h>

#include "maliput_multilane/make_road_curve_for_connection.h"

namespace maliput {
namespace multilane {

std::ostream& operator<<(std::ostream& out, const EndpointXy& endpoint_xy) {
  return out << "(x = " << endpoint_xy.x() << ", y = " << endpoint_xy.y() << ", heading = " << endpoint_xy.heading()
             << ")";
}

std::ostream& operator<<(std::ostream& out, const EndpointZ& endpoint_z) {
  return out << "(z = " << endpoint_z.z() << ", z_dot = " << endpoint_z.z_dot() << ", theta = " << endpoint_z.theta()
             << ", theta_dot = "
             << (endpoint_z.theta_dot().has_value() ? std::to_string(*endpoint_z.theta_dot())
                                                    : std::string("std::nullopt"))
             << ")";
}

std::ostream& operator<<(std::ostream& out, const Endpoint& endpoint) {
  return out << "(xy: " << endpoint.xy() << ", z: " << endpoint.z() << ")";
}

std::ostream& operator<<(std::ostream& out, const LineOffset& line_offset) {
  return out << "(length: " << line_offset.length() << ")";
}

std::ostream& operator<<(std::ostream& out, const ArcOffset& arc_offset) {
  return out << "(r: " << arc_offset.radius() << ", d_theta: " << arc_offset.d_theta() << ")";
}

struct Connection::Data {
  Type type{};
  std::string id;
  Endpoint start{};
  Endpoint end{};
  int num_lanes{};
  double r0{};
  double lane_width{};
  double left_shoulder{};
  double right_shoulder{};
  double r_min{};
  double r_max{};
  double linear_tolerance{};
  double scale_length{};
  ComputationPolicy computation_policy;
  std::unique_ptr<RoadCurve> road_curve;
  // Bits specific to type == kLine:
  double line_length{};
  // Bits specific to type == kArc:
  double radius{};
  double d_theta{};
  double theta0{};
  double cx{};
  double cy{};
};

Connection::~Connection() {}

Connection::Connection(const std::string& id, const Endpoint& start, const EndpointZ& end_z, int num_lanes, double r0,
                       double lane_width, double left_shoulder, double right_shoulder, const LineOffset& line_offset,
                       double linear_tolerance, double scale_length, ComputationPolicy computation_policy) {
  MALIPUT_DEMAND(num_lanes > 0);
  MALIPUT_DEMAND(lane_width >= 0);
  MALIPUT_DEMAND(left_shoulder >= 0);
  MALIPUT_DEMAND(right_shoulder >= 0);
  MALIPUT_DEMAND(linear_tolerance > 0.);
  MALIPUT_DEMAND(scale_length > 0.);
  MALIPUT_DEMAND(line_offset.length() > 0.);
  MALIPUT_DEMAND(start.z().theta_dot().has_value());
  MALIPUT_DEMAND(end_z.theta_dot().has_value());

  data_ = std::make_unique<Data>();
  data_->type = kLine;
  data_->id = id;
  data_->start = start;
  data_->num_lanes = num_lanes;
  data_->r0 = r0;
  data_->lane_width = lane_width;
  data_->left_shoulder = left_shoulder;
  data_->right_shoulder = right_shoulder;
  data_->r_min = r0 - lane_width / 2. - right_shoulder;
  data_->r_max = r0 + lane_width * (static_cast<double>(num_lanes - 1) + 0.5) + left_shoulder;
  MALIPUT_DEMAND(data_->r_max >= data_->r_min);

  data_->linear_tolerance = linear_tolerance;
  data_->scale_length = scale_length;
  data_->computation_policy = computation_policy;
  data_->line_length = line_offset.length();
  // Computes end Endpoint and RoadCurve.
  data_->end = Endpoint({start.xy().x() + data_->line_length * std::cos(start.xy().heading()),
                         start.xy().y() + data_->line_length * std::sin(start.xy().heading()), start.xy().heading()},
                        end_z);
  data_->road_curve = MakeRoadCurveFor(*this);
  // TODO(agalbachicar)  Modify Connection API to provide support for HBounds
  //                     once RoadCurve's children are capable of computing
  //                     singularities with it.
  MALIPUT_DEMAND(data_->road_curve->IsValid(data_->r_min, data_->r_max, {0., 0.}));
}

Connection::Connection(const std::string& id, const Endpoint& start, const EndpointZ& end_z, int num_lanes, double r0,
                       double lane_width, double left_shoulder, double right_shoulder, const ArcOffset& arc_offset,
                       double linear_tolerance, double scale_length, ComputationPolicy computation_policy) {
  MALIPUT_DEMAND(num_lanes > 0);
  MALIPUT_DEMAND(lane_width >= 0);
  MALIPUT_DEMAND(left_shoulder >= 0);
  MALIPUT_DEMAND(right_shoulder >= 0);
  MALIPUT_DEMAND(linear_tolerance > 0.);
  MALIPUT_DEMAND(scale_length > 0.);
  MALIPUT_DEMAND(arc_offset.radius() > 0);
  MALIPUT_DEMAND(start.z().theta_dot().has_value());
  MALIPUT_DEMAND(end_z.theta_dot().has_value());

  data_ = std::make_unique<Data>();
  data_->type = kArc;
  data_->id = id;
  data_->start = start;
  data_->num_lanes = num_lanes;
  data_->r0 = r0;
  data_->lane_width = lane_width;
  data_->left_shoulder = left_shoulder;
  data_->right_shoulder = right_shoulder;
  data_->r_min = r0 - lane_width / 2. - right_shoulder;
  data_->r_max = r0 + lane_width * (static_cast<double>(num_lanes - 1) + 0.5) + left_shoulder;
  MALIPUT_DEMAND(data_->r_max >= data_->r_min);
  data_->linear_tolerance = linear_tolerance;
  data_->scale_length = scale_length;
  data_->computation_policy = computation_policy;
  data_->radius = arc_offset.radius();
  data_->d_theta = arc_offset.d_theta();

  // Fills arc related parameters, computes end Endpoint and creates the
  // RoadCurve.
  data_->theta0 = start.xy().heading() - std::copysign(M_PI / 2., data_->d_theta);
  data_->cx = start.xy().x() - (data_->radius * std::cos(data_->theta0));
  data_->cy = start.xy().y() - (data_->radius * std::sin(data_->theta0));
  const double theta1 = data_->theta0 + data_->d_theta;
  data_->end = Endpoint({data_->cx + data_->radius * std::cos(theta1), data_->cy + data_->radius * std::sin(theta1),
                         start.xy().heading() + data_->d_theta},
                        end_z);
  data_->road_curve = MakeRoadCurveFor(*this);
  // TODO(agalbachicar)  Modify Connection API to provide support for HBounds
  //                     once RoadCurve's children are capable of computing
  //                     singularities with it.
  MALIPUT_DEMAND(data_->road_curve->IsValid(data_->r_min, data_->r_max, {0., 0.}));
}

Connection::Type Connection::type() const { return data_->type; }

const std::string& Connection::id() const { return data_->id; }

const Endpoint& Connection::start() const { return data_->start; }

const Endpoint& Connection::end() const { return data_->end; }

double Connection::line_length() const {
  MALIPUT_DEMAND(data_->type == kLine);
  return data_->line_length;
}

double Connection::radius() const {
  MALIPUT_DEMAND(data_->type == kArc);
  return data_->radius;
}

double Connection::d_theta() const {
  MALIPUT_DEMAND(data_->type == kArc);
  return data_->d_theta;
}

int Connection::num_lanes() const { return data_->num_lanes; }

double Connection::r0() const { return data_->r0; }

double Connection::lane_width() const { return data_->lane_width; }

double Connection::left_shoulder() const { return data_->left_shoulder; }

double Connection::right_shoulder() const { return data_->right_shoulder; }

double Connection::lane_offset(int lane_index) const {
  MALIPUT_DEMAND(lane_index >= 0 && lane_index < data_->num_lanes);
  return data_->r0 + data_->lane_width * static_cast<double>(lane_index);
}

double Connection::r_min() const { return data_->r_min; }

double Connection::r_max() const { return data_->r_max; }

double Connection::linear_tolerance() const { return data_->linear_tolerance; }

double Connection::scale_length() const { return data_->scale_length; }

ComputationPolicy Connection::computation_policy() const { return data_->computation_policy; }

Endpoint Connection::LaneStart(int lane_index) const {
  MALIPUT_DEMAND(lane_index >= 0 && lane_index < data_->num_lanes);
  const double r = lane_offset(lane_index);
  const math::Vector3 position = data_->road_curve->W_of_prh(0., r, 0.);
  const Rot3 rotation = data_->road_curve->Orientation(0., r, 0.);
  // Let t be the arc-length xy projection of the lane centerline and t_of_p be
  // a linear function of p (given that p <--> s is a linear relation too).
  // Given z_dot = ∂z/∂t, chain rule can be applied so:
  //
  // z_dot = ∂z/∂p * ∂p/∂t.
  //
  // The same applies to theta_dot.

  // Computes w_prime to obtain ∂z/∂p.
  const math::Vector3 w_prime = data_->road_curve->W_prime_of_prh(0., r, 0., data_->road_curve->Rabg_of_p(0.),
                                                                  data_->road_curve->elevation().f_dot_p(0.));
  // Computes ∂p/∂t based on Connection geometry type.

  // TODO(maddog-tri)  A (second-order?) contribution of theta_dot to ∂p/∂t is
  //                   being ignored.
  const double cos_superelevation = std::cos(data_->road_curve->superelevation().f_p(0.));
  const double t_max =
      data_->type == kLine
          ? data_->line_length
          : std::abs(data_->d_theta * (data_->radius - std::copysign(1., data_->d_theta) * r * cos_superelevation));
  // Given that ∂p/∂t = 1 / t_max.
  const double z_dot = w_prime.z() / t_max;
  // theta_dot is derivative with respect to t, but the reference curve t
  // coordinate. So, a ∂t_0/∂t_i is needed, being t_0 the reference curve
  // coordinate and t_i the arc-length xy projection for lane_index lane.
  const double theta_dot = data_->type == kLine
                               ? *data_->start.z().theta_dot()
                               : (*data_->start.z().theta_dot()) * std::abs(data_->d_theta * data_->radius) / t_max;
  return Endpoint({position[0], position[1], rotation.yaw()},
                  {position[2], z_dot, data_->start.z().theta(), theta_dot});
}

Endpoint Connection::LaneEnd(int lane_index) const {
  MALIPUT_DEMAND(lane_index >= 0 && lane_index < data_->num_lanes);
  const double r = lane_offset(lane_index);
  const math::Vector3 position = data_->road_curve->W_of_prh(1., r, 0.);
  const Rot3 rotation = data_->road_curve->Orientation(1., r, 0.);
  // Let t be the arc-length xy projection of the lane centerline and t_of_p be
  // a linear function of p (given that p <--> s is a linear relation too).
  // Given z_dot = ∂z/∂t, chain rule can be applied so:
  //
  // z_dot = ∂z/∂p * ∂p/∂t.
  //
  // The same applies to theta_dot.

  // Computes w_prime to obtain ∂z/∂p.
  const math::Vector3 w_prime = data_->road_curve->W_prime_of_prh(1., r, 0., data_->road_curve->Rabg_of_p(1.),
                                                                  data_->road_curve->elevation().f_dot_p(1.));
  // Computes ∂p/∂t based on Connection geometry type.

  // TODO(maddog-tri)  A (second-order?) contribution of theta_dot to ∂p/∂t is
  //                   being ignored.
  const double cos_superelevation = std::cos(data_->road_curve->superelevation().f_p(1.));
  const double t_max =
      data_->type == kLine
          ? data_->line_length
          : std::abs(data_->d_theta * (data_->radius - std::copysign(1., data_->d_theta) * r * cos_superelevation));
  // Given that ∂p/∂t = 1 / t_max.
  const double z_dot = w_prime.z() / t_max;
  // theta_dot is derivative with respect to t, but the reference curve t
  // coordinate. So, a ∂t_0/∂t_i is needed, being t_0 the reference curve
  // coordinate and t_i the arc-length xy projection for lane_index lane.
  const double theta_dot = data_->type == kLine
                               ? *data_->end.z().theta_dot()
                               : (*data_->end.z().theta_dot()) * std::abs(data_->d_theta * data_->radius) / t_max;
  return Endpoint({position[0], position[1], rotation.yaw()}, {position[2], z_dot, data_->end.z().theta(), theta_dot});
}

}  // namespace multilane
}  // namespace maliput
