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
#include "maliput_multilane/road_curve.h"

#include <algorithm>
#include <memory>

#include <maliput/common/maliput_abort.h>
#include <maliput/common/maliput_throw.h>
#include <maliput/common/maliput_unused.h>
#include <maliput/drake/integrator_configuration.h>

namespace maliput {
namespace multilane {

namespace {

// Arc length derivative function ds/dp = f(p; [r, h]) for numerical resolution
// of the s(p) mapping as an antiderivative computation (i.e. quadrature).
struct ArcLengthDerivativeFunction {
  // Constructs the arc length derivative function for the given @p road_curve.
  explicit ArcLengthDerivativeFunction(const RoadCurve* road_curve) : road_curve_(road_curve) {}

  // Computes the arc length derivative for the RoadCurve specified
  // at construction.
  //
  // @param p The parameterization value to evaluate the derivative at.
  // @param k The parameter vector, containing r and h coordinates respectively.
  // @return The arc length derivative value at the specified point.
  // @pre The given parameter vector @p k is bi-dimensional (holding r and h
  //      coordinates only).
  // @throws std::logic_error if preconditions are not met.
  double operator()(const double& p, const math::Vector2& k) const {
    return road_curve_->W_prime_of_prh(p, k[0], k[1], road_curve_->Rabg_of_p(p), road_curve_->elevation().f_dot_p(p))
        .norm();
  }

 private:
  // Associated RoadCurve instance.
  const RoadCurve* road_curve_;
};

// Inverse arc length ODE function dp/ds = f(s, p; [r, h]) for numerical
// resolution of the p(s) mapping as an scalar initial value problem for
// a given RoadCurve.
struct InverseArcLengthODEFunction {
  // Constructs an inverse arc length ODE for the given @p road_curve.
  explicit InverseArcLengthODEFunction(const RoadCurve* road_curve) : road_curve_(road_curve) {}

  // Computes the inverse arc length derivative for the RoadCurve specified
  // at construction.
  //
  // @param s The arc length to evaluate the derivative at.
  // @param p The parameterization value to evaluate the derivative at.
  // @param k The parameter vector, containing r and h coordinates respectively.
  // @return The inverse arc length derivative value at the specified point.
  // @pre The given parameter vector @p k is bi-dimensional (holding r and h
  //      coordinates only).
  // @throws std::logic_error if preconditions are not met.
  double operator()(const double& s, const double& p, const math::Vector2& k) {
    maliput::common::unused(s);
    return 1.0 /
           road_curve_->W_prime_of_prh(p, k[0], k[1], road_curve_->Rabg_of_p(p), road_curve_->elevation().f_dot_p(p))
               .norm();
  }

 private:
  // Associated RoadCurve instance.
  const RoadCurve* road_curve_;
};

}  // namespace

RoadCurve::RoadCurve(double linear_tolerance, double scale_length, const CubicPolynomial& elevation,
                     const CubicPolynomial& superelevation, ComputationPolicy computation_policy)
    : scale_length_(scale_length),
      linear_tolerance_(linear_tolerance),
      elevation_(elevation),
      superelevation_(superelevation),
      computation_policy_(computation_policy) {
  // Enforces preconditions.
  MALIPUT_THROW_UNLESS(scale_length > 0.);
  MALIPUT_THROW_UNLESS(linear_tolerance > 0.);
  // Sets default parameter value at the beginning of the
  // curve to 0 by default.
  const double initial_p_value = 0.0;
  // Sets default arc length at the beginning of the curve
  // to 0 by default.
  const double initial_s_value = 0.0;
  // Sets default r and h coordinates to 0 by default.
  const math::Vector2 default_parameters(0., 0.);

  // Relative tolerance in path length is roughly bounded by e/L, where e is
  // the linear tolerance and L is the scale length. This can be seen by
  // considering straight path one scale length (or spatial period) long, and
  // then another path, whose deviation from the first is a sine function with
  // the same period and amplitude equal to the specified tolerance. The
  // difference in path length is bounded by 4e and the relative error is thus
  // bounded by 4e/L.
  relative_tolerance_ = linear_tolerance_ / scale_length_;

  const struct maliput::drake::IntegratorConfiguration arc_length_integrator_config {
    initial_p_value,        /* parameter_lower_bound */
        initial_s_value,    /* image_lower_bound */
        default_parameters, /* k */
        // Sets `s_from_p`'s integration accuracy and step sizes. Said steps
        // should not be too large, because that could make accuracy control
        // fail, nor too small to avoid wasting cycles. The nature of the
        // problem at hand varies with the parameterization of the RoadCurve,
        // and so will optimal step sizes (in terms of their efficiency vs.
        // accuracy balance). However, for the time being, the following
        // constants (considering 0.0 <= p <= 1.0) work well as a heuristic
        // approximation to appropriate step sizes.
        0.1,                       /* initial_step_size_target */
        1.0,                       /* maximum_step_size */
        relative_tolerance_ * 1e-2 /* target_accuracy */
  };

  const struct maliput::drake::IntegratorConfiguration inverse_arc_length_integrator_config {
    initial_p_value,        /* parameter_lower_bound */
        initial_s_value,    /* image_lower_bound */
        default_parameters, /* k */
        // Sets `p_from_s`'s integration accuracy and step sizes. Said steps
        // should not be too large, because that could make accuracy control
        // fail, nor too small to avoid wasting cycles. The nature of the
        // problem at hand varies with the shape of the RoadCurve, and so will
        // optimal step sizes (in terms of their efficiency vs. accuracy balance).
        // However, for the time being, the following proportions of the scale
        // length work well as a heuristic approximation to appropriate step sizes.
        0.1 * scale_length, /* initial_step_size_target */
        scale_length,       /* maximum_step_size */
        relative_tolerance_ /* target_accuracy */
  };

  // Instantiates s(p) and p(s) mappings with default values.
  s_from_p_func_ = std::make_unique<maliput::drake::ArcLengthIntegrator>(ArcLengthDerivativeFunction(this),
                                                                         arc_length_integrator_config);
  p_from_s_ivp_ = std::make_unique<maliput::drake::InverseArcLengthIntegrator>(InverseArcLengthODEFunction(this),
                                                                               inverse_arc_length_integrator_config);
}

bool RoadCurve::AreFastComputationsAccurate(double r) const {
  // When superelevation() has no influence on the curve's
  // geometry and elevation() is at most linear along the curve,
  // known analytical expressions are accurate.
  return ((r == 0.0 || superelevation().is_zero()) && elevation().order() <= 1);
}

std::function<double(double)> RoadCurve::OptimizeCalcSFromP(double r) const {
  MALIPUT_THROW_UNLESS(CalcMinimumRadiusAtOffset(r) > 0.0);
  const double absolute_tolerance = relative_tolerance_ * 1.;
  if (computation_policy() == ComputationPolicy::kPreferAccuracy && !AreFastComputationsAccurate(r)) {
    // Populates parameter vector with (r, h=0.0) coordinate values.
    return s_from_p_func_->IntegralFunction(0.0, 1.0, math::Vector2(r, 0.0), absolute_tolerance);
  }
  return [this, r, absolute_tolerance](double p) {
    // Saturates p to lie within the [0., 1.] interval.
    const double saturated_p = std::min(std::max(p, 0.), 1.);
    MALIPUT_THROW_UNLESS(std::abs(saturated_p - p) < absolute_tolerance);
    return this->FastCalcSFromP(p, r);
  };
}

double RoadCurve::CalcSFromP(double p, double r) const {
  // Populates parameter vector with (r, h=0.0) coordinate values.
  return s_from_p_func_->Evaluate(p, math::Vector2(r, 0.0));
}

std::function<double(double)> RoadCurve::OptimizeCalcPFromS(double r) const {
  MALIPUT_THROW_UNLESS(CalcMinimumRadiusAtOffset(r) > 0.0);
  const double full_length = CalcSFromP(1., r);
  const double absolute_tolerance = relative_tolerance_ * full_length;
  if (computation_policy() == ComputationPolicy::kPreferAccuracy && !AreFastComputationsAccurate(r)) {
    return p_from_s_ivp_->InverseFunction(0.0, full_length, math::Vector2(r, 0.0), absolute_tolerance);
  }
  return [this, r, full_length, absolute_tolerance](double s) {
    // Saturates s to lie within the [0., full_length] interval.
    const double saturated_s = std::min(std::max(s, 0.), full_length);
    MALIPUT_THROW_UNLESS(std::abs(saturated_s - s) < absolute_tolerance);
    return this->FastCalcPFromS(s, r);
  };
}

double RoadCurve::CalcGPrimeAsUsedForCalcSFromP(double p) const {
  if (computation_policy() == ComputationPolicy::kPreferSpeed) {
    return elevation().fake_gprime(p);
  }
  return elevation().f_dot_p(p);
}

math::Vector3 RoadCurve::W_of_prh(double p, double r, double h) const {
  // Calculates z (elevation) of (p,0,0).
  const double z = elevation().f_p(p) * l_max();
  // Calculates x,y of (p,0,0).
  const math::Vector2 xy = xy_of_p(p);
  // Calculates orientation of (p,r,h) basis at (p,0,0).
  const Rot3 ypr = Rabg_of_p(p);
  // Rotates (0,r,h) and sums with mapped (p,0,0).
  return ypr.apply({0., r, h}) + math::Vector3(xy.x(), xy.y(), z);
}

math::Vector3 RoadCurve::W_prime_of_prh(double p, double r, double h, const Rot3& Rabg, double g_prime) const {
  const math::Vector2 G_prime = xy_dot_of_p(p);

  const Rot3& R = Rabg;
  const double alpha = R.roll();
  const double beta = R.pitch();
  const double gamma = R.yaw();

  const double ca = std::cos(alpha);
  const double cb = std::cos(beta);
  const double cg = std::cos(gamma);
  const double sa = std::sin(alpha);
  const double sb = std::sin(beta);
  const double sg = std::sin(gamma);

  // Evaluate dα/dp, dβ/dp, dγ/dp...
  const double d_alpha = superelevation().f_dot_p(p) * l_max();
  const double d_beta = -cb * cb * elevation().f_ddot_p(p);
  const double d_gamma = heading_dot_of_p(p);

  // Recall that W is the lane-to-world transform, defined by
  //   (x,y,z)  = W(p,r,h) = (G(p), Z(p)) + R_αβγ*(0,r,h)
  // where G is the reference curve, Z is the elevation profile, and R_αβγ is
  // a rotation matrix derived from reference curve (heading), elevation,
  // and superelevation.
  //
  // Thus, ∂W/∂p = (∂G(p)/∂p, ∂Z(p)/∂p) + (∂R_αβγ/∂p)*(0,r,h), where
  //
  //   ∂G(p)/∂p = G'(p)
  //
  //   ∂Z(p)/∂p = l_max * (z / l_max) = l_max * g'(p)
  //
  //   ∂R_αβγ/∂p = (∂R_αβγ/∂α ∂R_αβγ/∂β ∂R_αβγ/∂γ)*(dα/dp, dβ/dp, dγ/dp)
  return math::Vector3(G_prime.x(), G_prime.y(), l_max() * g_prime) +

         math::Vector3((((sa * sg) + (ca * sb * cg)) * r + ((ca * sg) - (sa * sb * cg)) * h),
                       (((-sa * cg) + (ca * sb * sg)) * r - ((ca * cg) + (sa * sb * sg)) * h),
                       ((ca * cb) * r + (-sa * cb) * h)) *
             d_alpha +

         math::Vector3(((sa * cb * cg) * r + (ca * cb * cg) * h), ((sa * cb * sg) * r + (ca * cb * sg) * h),
                       ((-sa * sb) * r - (ca * sb) * h)) *
             d_beta +

         math::Vector3((((-ca * cg) - (sa * sb * sg)) * r + ((+sa * cg) - (ca * sb * sg)) * h),
                       (((-ca * sg) + (sa * sb * cg)) * r + ((sa * sg) + (ca * sb * cg)) * h), 0) *
             d_gamma;
}

Rot3 RoadCurve::Rabg_of_p(double p) const {
  return Rot3(superelevation().f_p(p) * l_max(), -std::atan(elevation().f_dot_p(p)), heading_of_p(p));
}

Rot3 RoadCurve::Orientation(double p, double r, double h) const {
  // Calculate orientation of (s,r,h) basis at (s,0,0).
  const Rot3 Rabg = Rabg_of_p(p);
  const double real_g_prime = elevation().f_dot_p(p);

  // Calculate s,r basis vectors at (s,r,h)...
  const math::Vector3 s_hat = s_hat_of_prh(p, r, h, Rabg, real_g_prime);
  const math::Vector3 r_hat = r_hat_of_Rabg(Rabg);
  // ...and then derive orientation from those basis vectors.
  //
  // (s_hat  r_hat  h_hat) is an orthonormal basis, obtained by rotating the
  // (x_hat  y_hat  z_hat) basis by some R-P-Y rotation; in this case, we know
  // the value of (s_hat  r_hat  h_hat) (w.r.t. 'xyz' world frame), so we are
  // trying to recover the roll/pitch/yaw.  Since (x_hat  y_hat  z_hat) is an
  // identity matrix (e.g., x_hat = column vector (1, 0, 0), etc), then
  // (s_hat  r_hat  h_hat) equals the R-P-Y matrix itself.
  // If we define a = alpha = roll, b = beta = pitch, g = gamma = yaw,
  // then s_hat is the first column of the rotation, r_hat is the second:
  //   s_hat = (cb * cg, cb * sg, - sb)
  //   r_hat = (- ca * sg + sa * sb * cg, ca * cg + sa * sb * sg, sa * cb)
  // We solve the above for a, b, g.
  const double gamma = std::atan2(s_hat.y(), s_hat.x());
  const double beta = std::atan2(-s_hat.z(), math::Vector2(s_hat.x(), s_hat.y()).norm());
  const double cb = std::cos(beta);
  const double alpha = std::atan2(r_hat.z() / cb, ((r_hat.y() * s_hat.x()) - (r_hat.x() * s_hat.y())) / cb);
  return {alpha, beta, gamma};
}

math::Vector3 RoadCurve::s_hat_of_prh(double p, double r, double h, const Rot3& Rabg, double g_prime) const {
  const math::Vector3 W_prime = W_prime_of_prh(p, r, h, Rabg, g_prime);
  return W_prime * (1.0 / W_prime.norm());
}

math::Vector3 RoadCurve::r_hat_of_Rabg(const Rot3& Rabg) const { return Rabg.apply({0., 1., 0.}); }

}  // namespace multilane
}  // namespace maliput
