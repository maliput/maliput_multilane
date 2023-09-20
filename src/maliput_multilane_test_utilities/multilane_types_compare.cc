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
#include "maliput_multilane_test_utilities/multilane_types_compare.h"

#include <cmath>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace maliput {
namespace multilane {
namespace test {

::testing::AssertionResult IsEndpointXyClose(const EndpointXy& xy1, const EndpointXy& xy2, double linear_tolerance,
                                             double angular_tolerance) {
  bool fails = false;
  std::string error_message{};
  double delta = std::abs(xy1.x() - xy2.x());
  if (delta > linear_tolerance) {
    fails = true;
    std::stringstream ss;
    ss << "EndpointXys are different at x coordinate.\n";
    ss << "xy1.x(): " << xy1.x() << " vs. xy2.x(): " << xy2.x() << ", diff = " << delta
       << ", linear tolerance = " << linear_tolerance << "\n";
    error_message += ss.str();
  }
  delta = std::abs(xy1.y() - xy2.y());
  if (delta > linear_tolerance) {
    fails = true;
    std::stringstream ss;
    ss << "EndpointXys are different at y coordinate.\n";
    ss << "xy1.y(): " << xy1.y() << " vs. xy2.y(): " << xy2.y() << ", diff = " << delta
       << ", linear tolerance = " << linear_tolerance << "\n";
    error_message += ss.str();
  }
  delta = std::abs(xy1.heading() - xy2.heading());
  if (delta > angular_tolerance) {
    fails = true;
    std::stringstream ss;
    ss << "EndpointXys are different at heading angle.\n";
    ss << "xy1.heading(): " << xy1.heading() << " vs.xy2.heading(): " << xy2.heading() << ", diff = " << delta
       << ", angular tolerance = " << angular_tolerance << "\n";
    error_message += ss.str();
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  std::stringstream ss;
  ss << "xy1 =\n"
     << xy1 << "\nis approximately equal to xy2 =\n"
     << xy2 << "\n"
     << "linear tolerance = " << linear_tolerance << ", angular tolerance = " << angular_tolerance;
  return ::testing::AssertionSuccess() << ss.str();
}

::testing::AssertionResult IsEndpointZClose(const EndpointZ& z1, const EndpointZ& z2, double linear_tolerance,
                                            double angular_tolerance) {
  bool fails = false;
  std::string error_message{};
  double delta = std::abs(z1.z() - z2.z());
  if (delta > linear_tolerance) {
    fails = true;
    std::stringstream ss;
    ss << "EndpointZ are different at z coordinate. ";
    ss << "z1.z(): " << z1.z() << " vs z2.z(): " << z2.z() << ", diff = " << delta
       << ", linear tolerance = " << linear_tolerance << "\n";
    error_message += ss.str();
  }
  delta = std::abs(z1.z_dot() - z2.z_dot());
  if (delta > linear_tolerance) {
    fails = true;
    std::stringstream ss;
    ss << "EndpointZ are different at z_dot coordinate. ";
    ss << "z1.z_dot(): " << z1.z_dot() << " vs z2.z_dot(): " << z2.z_dot() << ", diff = " << delta
       << ", linear tolerance = " << linear_tolerance << "\n";
    error_message += ss.str();
  }
  delta = std::abs(z1.theta() - z2.theta());
  if (delta > angular_tolerance) {
    fails = true;
    std::stringstream ss;
    ss << "EndpointZ are different at theta angle. ";
    ss << "z1.theta(): " << z1.theta() << " vs z2.theta(): " << z2.theta() << ", diff = " << delta
       << ", angular tolerance = " << angular_tolerance << "\n";
    error_message += ss.str();
  }
  if (z1.theta_dot().has_value() && z2.theta_dot().has_value()) {
    delta = std::abs(*z1.theta_dot() - *z2.theta_dot());
    if (delta > angular_tolerance) {
      fails = true;
      std::stringstream ss;
      ss << "EndpointZ are different at theta_dot. ";
      ss << "z1.theta_dot(): " << *z1.theta_dot() << " vs z2.theta_dot(): " << *z2.theta_dot() << ", diff = " << delta
         << ", angular tolerance = " << angular_tolerance << "\n";
      error_message += ss.str();
    }
  } else if (z1.theta_dot().has_value() && !z2.theta_dot().has_value()) {
    fails = true;
    error_message +=
        "EndpointZ are different at theta_dot. "
        "z1.theta_dot() has value but z2.theta_dot does not.\n";
  } else if (!z1.theta_dot().has_value() && z2.theta_dot().has_value()) {
    fails = true;
    error_message +=
        "EndpointZ are different at theta_dot. "
        "z1.theta_dot() does not have a value but "
        "z2.theta_dot does.\n";
  }

  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  std::stringstream ss;
  ss << "z1 =\n"
     << z1 << "\nis approximately equal to z2 =\n"
     << z2 << "\n"
     << "linear tolerance = " << linear_tolerance << ", angular tolerance = " << angular_tolerance;
  return ::testing::AssertionSuccess() << ss.str();
}

::testing::AssertionResult IsEndpointClose(const Endpoint& p1, const Endpoint& p2, double linear_tolerance,
                                           double angular_tolerance) {
  bool fails = false;
  std::string error_message{};

  const ::testing::AssertionResult endpoint_xy_comparison =
      IsEndpointXyClose(p1.xy(), p2.xy(), linear_tolerance, angular_tolerance);
  if (!endpoint_xy_comparison) {
    fails = true;
    error_message +=
        "Endpoint p1 is different from p2 at EndpointXy. [" + std::string(endpoint_xy_comparison.message()) + "]\n";
  }
  const ::testing::AssertionResult endpoint_z_comparison =
      IsEndpointZClose(p1.z(), p2.z(), linear_tolerance, angular_tolerance);
  if (!endpoint_z_comparison) {
    fails = true;
    error_message +=
        "Endpoint p1 is different from p2 at EndpointZ. [" + std::string(endpoint_z_comparison.message()) + "]\n";
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  std::stringstream ss;
  ss << "p1 =\n"
     << p1 << "\nis approximately equal to p2 =\n"
     << p2 << "\n"
     << "linear tolerance = " << linear_tolerance << ", angular tolerance = " << angular_tolerance;
  return ::testing::AssertionSuccess() << ss.str();
}

::testing::AssertionResult IsArcOffsetClose(const ArcOffset& arc_offset1, const ArcOffset& arc_offset2,
                                            double linear_tolerance, double angular_tolerance) {
  bool fails = false;
  std::string error_message{};
  double delta = std::abs(arc_offset1.radius() - arc_offset2.radius());
  if (delta > linear_tolerance) {
    fails = true;
    std::stringstream ss;
    ss << "ArcOffset are different at radius. ";
    ss << "arc_offset1.radius(): " << arc_offset1.radius() << " vs. arc_offset2.radius(): " << arc_offset2.radius()
       << ", diff = " << delta << ", tolerance = " << linear_tolerance << "\n";
    error_message += ss.str();
  }
  delta = std::abs(arc_offset1.d_theta() - arc_offset2.d_theta());
  if (delta > angular_tolerance) {
    fails = true;
    std::stringstream ss;
    ss << "ArcOffset are different at d_theta. ";
    ss << "arc_offset1.d_theta(): " << arc_offset1.d_theta() << " vs. arc_offset2.d_theta(): " << arc_offset2.d_theta()
       << ", diff = " << delta << ", tolerance = " << angular_tolerance << "\n";
    error_message += ss.str();
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  std::stringstream ss;
  ss << "arc_offset1 =\n"
     << arc_offset1 << "\nis approximately equal to arc_offset2 =\n"
     << arc_offset2 << "\n"
     << "linear tolerance = " << linear_tolerance << "\nand angular tolerance = " << angular_tolerance;
  return ::testing::AssertionSuccess() << ss.str();
}

Matcher<const api::HBounds&> Matches(const api::HBounds& elevation_bounds, double tolerance) {
  return MakeMatcher(new HBoundsMatcher(elevation_bounds, tolerance));
}

Matcher<const ArcOffset&> Matches(const ArcOffset& arc_offset, double linear_tolerance, double angular_tolerance) {
  return MakeMatcher(new ArcOffsetMatcher(arc_offset, linear_tolerance, angular_tolerance));
}

Matcher<const LineOffset&> Matches(const LineOffset& line_offset, double tolerance) {
  return MakeMatcher(new LineOffsetMatcher(line_offset, tolerance));
}

Matcher<const LaneLayout&> Matches(const LaneLayout& lane_layout, double tolerance) {
  return MakeMatcher(new LaneLayoutMatcher(lane_layout, tolerance));
}

Matcher<const StartReference::Spec&> Matches(const StartReference::Spec& start_reference, double linear_tolerance,
                                             double angular_tolerance) {
  return MakeMatcher(new StartReferenceSpecMatcher(start_reference, linear_tolerance, angular_tolerance));
}

Matcher<const StartLane::Spec&> Matches(const StartLane::Spec& start_lane, double linear_tolerance,
                                        double angular_tolerance) {
  return MakeMatcher(new StartLaneSpecMatcher(start_lane, linear_tolerance, angular_tolerance));
}

Matcher<const EndReference::Spec&> Matches(const EndReference::Spec& end_reference, double linear_tolerance,
                                           double angular_tolerance) {
  return MakeMatcher(new EndReferenceSpecMatcher(end_reference, linear_tolerance, angular_tolerance));
}

Matcher<const EndLane::Spec&> Matches(const EndLane::Spec& end_lane, double linear_tolerance,
                                      double angular_tolerance) {
  return MakeMatcher(new EndLaneSpecMatcher(end_lane, linear_tolerance, angular_tolerance));
}

}  // namespace test
}  // namespace multilane
}  // namespace maliput
