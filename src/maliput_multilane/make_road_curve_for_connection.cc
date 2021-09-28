#include "maliput_multilane/make_road_curve_for_connection.h"

#include <cmath>

#include <drake/common/eigen_types.h>
#include <maliput/common/maliput_abort.h>

#include "maliput_multilane/arc_road_curve.h"
#include "maliput_multilane/computation_policy.h"
#include "maliput_multilane/cubic_polynomial.h"
#include "maliput_multilane/line_road_curve.h"

namespace maliput {
namespace multilane {
namespace {
// Construct a CubicPolynomial such that:
//    f(0) = Y0 / dX           f'(0) = Ydot0
//    f(1) = (Y0 + dY) / dX    f'(1) = Ydot1
//
// This is equivalent to taking a cubic polynomial g such that:
//    g(0) = Y0          g'(0) = Ydot0
//    g(dX) = Y0 + dY    g'(1) = Ydot1
// and isotropically scaling it (scale both axes) by a factor of 1/dX
CubicPolynomial MakeCubic(double dX, double Y0, double dY, double Ydot0, double Ydot1) {
  return CubicPolynomial(Y0 / dX, Ydot0, (3. * dY / dX) - (2. * Ydot0) - Ydot1, Ydot0 + Ydot1 - (2. * dY / dX));
}
}  // namespace

std::unique_ptr<RoadCurve> MakeRoadCurveFor(const Connection& connection) {
  switch (connection.type()) {
    case Connection::kLine: {
      const drake::Vector2<double> xy0(connection.start().xy().x(), connection.start().xy().y());
      const drake::Vector2<double> dxy(connection.end().xy().x() - connection.start().xy().x(),
                                       connection.end().xy().y() - connection.start().xy().y());
      const CubicPolynomial elevation(MakeCubic(dxy.norm(), connection.start().z().z(),
                                                connection.end().z().z() - connection.start().z().z(),
                                                connection.start().z().z_dot(), connection.end().z().z_dot()));
      const CubicPolynomial superelevation(MakeCubic(
          dxy.norm(), connection.start().z().theta(), connection.end().z().theta() - connection.start().z().theta(),
          *connection.start().z().theta_dot(), *connection.end().z().theta_dot()));
      return std::make_unique<LineRoadCurve>(xy0, dxy, elevation, superelevation, connection.linear_tolerance(),
                                             connection.scale_length(), connection.computation_policy());
    };
    case Connection::kArc: {
      const double arc_length = connection.radius() * std::abs(connection.d_theta());
      const double theta0 = connection.start().xy().heading() - std::copysign(M_PI / 2., connection.d_theta());
      const drake::Vector2<double> center(connection.start().xy().x() - (connection.radius() * std::cos(theta0)),
                                          connection.start().xy().y() - (connection.radius() * std::sin(theta0)));
      const CubicPolynomial elevation(MakeCubic(arc_length, connection.start().z().z(),
                                                connection.end().z().z() - connection.start().z().z(),
                                                connection.start().z().z_dot(), connection.end().z().z_dot()));
      const CubicPolynomial superelevation(MakeCubic(
          arc_length, connection.start().z().theta(), connection.end().z().theta() - connection.start().z().theta(),
          *connection.start().z().theta_dot(), *connection.end().z().theta_dot()));
      return std::make_unique<ArcRoadCurve>(center, connection.radius(), theta0, connection.d_theta(), elevation,
                                            superelevation, connection.linear_tolerance(), connection.scale_length(),
                                            connection.computation_policy());
    };
  }
  MALIPUT_ABORT_MESSAGE("type_ is neither Connection::kArc nor Connection::kLine.");
}

}  // namespace multilane
}  // namespace maliput
