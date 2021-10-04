/* clang-format off to disable clang-format-includes */
#include "maliput_multilane/road_curve.h"
/* clang-format on */

#include <memory>
#include <ostream>
#include <tuple>
#include <utility>

#include <fmt/format.h>
#include <gtest/gtest.h>
#include <maliput/common/maliput_throw.h>
#include <maliput/math/vector.h>

#include "maliput_multilane/arc_road_curve.h"
#include "maliput_multilane/cubic_polynomial.h"
#include "maliput_multilane/line_road_curve.h"
#include "maliput_multilane/road_curve.h"

namespace maliput {
namespace multilane {
namespace {

// Approximates a path length lower bound for the given @p road_curve
// from @p p_0 to @p p_1, for constant @p r and @p h offsets, by
// computing the same integral for a 2^@p k_order linear approximation.
//
// @param rc The RoadCurve to compute path length for.
// @param p_0 The lower integration bound for the p coordinate.
// @param p_1 The upper integration bound for the p coordinate.
// @param r The r coordinate offset.
// @param h The h coordinate offset.
// @param k_order Order k of the linear approximation, i.e. 2^k segments
//                are used in the approximation.
// @param maximum_step A mutable reference that, if given (can be nullptr), is
//                     set to the maximum step length used in the computation.
// @pre Given upper integration bound @p p_1 is less than or equal to 1.
// @pre Given upper integration bound @p p_1 is greater than or equal to
//      the given lower integration bound @p p_0.
// @pre Given lower integration bound @p p_0 is greater than or equal to 0.
// @pre Given @p k_order for the linear approximation is a non-negative number.
// @throws std::runtime_error if preconditions are not met.
double BruteForcePathLengthIntegral(const RoadCurve& rc, double p_0, double p_1, double r, double h, int k_order,
                                    double* maximum_step) {
  MALIPUT_THROW_UNLESS(0. <= p_0);
  MALIPUT_THROW_UNLESS(p_0 <= p_1);
  MALIPUT_THROW_UNLESS(p_1 <= 1.);
  MALIPUT_THROW_UNLESS(k_order >= 0);
  double length = 0.0;
  const double d_p = (p_1 - p_0);
  const int iterations = std::pow(2, k_order);
  if (maximum_step != nullptr) {
    *maximum_step = -std::numeric_limits<double>::infinity();
  }
  // Splits the [p_0, p_1] interval in 2^k intervals, computes the positions
  // in the global frame for each interval boundary and sums up the path lengths
  // of the segments in the global frame that correspond to each one of those
  // intervals.
  math::Vector3 geo_position_at_prev_p = rc.W_of_prh(p_0, r, h);
  for (int i = 1; i <= iterations; ++i) {
    const double p = p_0 + d_p * static_cast<double>(i) / iterations;
    const math::Vector3 geo_position_at_p = rc.W_of_prh(p, r, h);
    const double ith_step_length = (geo_position_at_p - geo_position_at_prev_p).norm();
    if (maximum_step != nullptr) {
      // Keep track of the maximum step taken.
      *maximum_step = std::max(*maximum_step, ith_step_length);
    }
    length += ith_step_length;
    geo_position_at_prev_p = geo_position_at_p;
  }
  return length;
}

// Approximates the path length of a given @p road_curve from @p p_0 to @p p_1,
// for constant @p r and @p h offsets, to within specified @p tolerance.
//
// To ensure the error falls within @p tolerance, a path length lower bound
// is used (see BruteForcePathLengthIntegral()). If the curve is split in pieces
// no longer than the scale length of the curve, assuming the curve is well
// behaved, each one of the pieces can roughly be approximated as a constant
// curvature arc of radius R, subtending an angle θ. Then, let E(k) be the kth
// order approximation computed as the length of the chord that connects the arc
// endpoints, E(k+1) be the (k+1)th order approximation computed as the sum
// of the lengths of the resulting chords after an arc bisection and E(∞) be the
// true path length. It can be shown that: E(∞) - E(k+1) <= E(k+1) - E(k).
//
// TODO(hidmic): Compute a path length upper bound to ensure the approximation
// is within tolerance.
//
// @param rc The RoadCurve to compute path length for.
// @param p_0 The lower integration bound for the p coordinate.
// @param p_1 The upper integration bound for the p coordinate.
// @param r The r coordinate offset.
// @param h The h coordinate offset.
// @param tolerance The tolerance for the approximation, in the absolute error
//                  sense.
// @param k_order_hint A mutable reference to the order k of the linear
//                     approximation that, if given (can be nullptr) it's used
//                     as hint on call and it's updated to the actually required
//                     order necessary to achieve the specified tolerance on
//                     return.
// @pre Given upper integration bound @p p_1 is less than or equal to 1.
// @pre Given upper integration bound @p p_1 is greater than or equal to
//      the given lower integration bound @p p_0.
// @pre Given lower integration bound @p p_0 is greater than or equal to 0.
// @pre Given tolerance is a positive real number.
// @pre If given, the order suggested by @p k_order_hint is a non-negative
//      number.
// @throws std::runtime_error if preconditions are not met.
double AdaptiveBruteForcePathLengthIntegral(const RoadCurve& rc, double p_0, double p_1, double r, double h,
                                            double tolerance, int* k_order_hint) {
  MALIPUT_THROW_UNLESS(tolerance > 0.);
  const double kInfinity = std::numeric_limits<double>::infinity();
  // Zero initializes the current k order unless a hint was provided.
  int k_order = k_order_hint != nullptr ? *k_order_hint : 0;
  double k_order_maximum_step = kInfinity;
  // Computes the k-order path length approximation.
  double k_order_path_length = BruteForcePathLengthIntegral(rc, p_0, p_1, r, h, k_order, &k_order_maximum_step);
  // Estimates the error of a k-order approximation, increasing k
  // until said error falls within the specified tolerance.
  while (true) {
    double k_plus_1_order_maximum_step = kInfinity;
    // Computes the k+1-order path length approximation.
    const double k_plus_1_order_path_length =
        BruteForcePathLengthIntegral(rc, p_0, p_1, r, h, k_order + 1, &k_plus_1_order_maximum_step);
    // Estimates the error of the k-order path length approximation
    // by comparing it with the k+1-order one.
    const double k_order_error = std::abs(k_plus_1_order_path_length - k_order_path_length);
    // Not only the estimated error must be within tolerance but
    // also the maximum step taken by the k-order approximation
    // must be within a scale length to ensure it is valid (see
    // AdaptiveBruteForcePathLengthIntegral() function documentation).
    if (k_order_maximum_step < rc.scale_length() && k_order_error < tolerance) {
      break;
    }
    k_order_path_length = k_plus_1_order_path_length;
    k_order_maximum_step = k_plus_1_order_maximum_step;
    k_order += 1;
  }
  // Update k-order hint with actual k-order required to achieve
  // the desired accuracy.
  if (k_order_hint != nullptr) *k_order_hint = k_order;
  return k_order_path_length;
}

// Checks brute force integral computations against known
// path length arc road curves.
GTEST_TEST(BruteForceIntegralTest, ArcRoadCurvePathLength) {
  const double kAccuracy{1e-12};

  const double kRadius{10.0};
  const double kTheta0{M_PI / 4.0};
  const double kTheta1{3.0 * M_PI / 4.0};
  const double kDTheta{kTheta1 - kTheta0};
  const math::Vector2 kCenter{10.0, 10.0};
  const double kLinearTolerance{0.01};
  const double kScaleLength{1.};
  const ComputationPolicy kComputationPolicy{ComputationPolicy::kPreferAccuracy};
  const CubicPolynomial zp(0., 0., 0., 0.);
  const double kP0{0.};
  const double kP1{1.};
  const double kR{0.};
  const double kH{0.};

  const ArcRoadCurve rc(kCenter, kRadius, kTheta0, kDTheta, zp, zp, kLinearTolerance, kScaleLength, kComputationPolicy);

  // A k = 0 order approximation uses a single segment, as n = 2^k,
  // where n is the segment count.
  double maximum_step = 0;
  const double path_length_zero_order_approx = BruteForcePathLengthIntegral(rc, kP0, kP1, kR, kH, 0, &maximum_step);
  const double path_length_zero_order = std::sin(M_PI / 4.) * kRadius * 2.;
  EXPECT_NEAR(path_length_zero_order_approx, path_length_zero_order, kAccuracy);
  EXPECT_NEAR(maximum_step, path_length_zero_order, kAccuracy);

  int k_order_hint = 0;
  const double tolerance = .01 * rc.OptimizeCalcSFromP(0.)(1.);
  const double path_length_adaptive_approx =
      AdaptiveBruteForcePathLengthIntegral(rc, kP0, kP1, kR, kH, tolerance, &k_order_hint);
  EXPECT_NEAR(path_length_adaptive_approx, kRadius * M_PI / 2., tolerance);
}

// A test fixture for RoadCurve computation accuracy tests.
class RoadCurveAccuracyTest : public ::testing::TestWithParam<std::shared_ptr<RoadCurve>> {};

// Checks that optimized path length computations are within tolerance.
TEST_P(RoadCurveAccuracyTest, PathLengthComputationAccuracy) {
  const double kMinimumP = 0.;
  const double kMaximumP = 1.;
  const double kPStep = 0.2;

  const double kMinimumR = -5.0;
  const double kMaximumR = 5.0;
  const double kRStep = 2.5;

  const double kH = 0.0;

  std::shared_ptr<RoadCurve> road_curve = GetParam();
  const double kTolerance = road_curve->linear_tolerance() / road_curve->scale_length();
  for (double r = kMinimumR; r <= kMaximumR; r += kRStep) {
    int k_order = 0;
    std::function<double(double)> s_from_p_at_r = road_curve->OptimizeCalcSFromP(r);
    for (double p = kMinimumP; p <= kMaximumP; p += kPStep) {
      const double k_order_s_approximation = AdaptiveBruteForcePathLengthIntegral(
          *road_curve, kMinimumP, p, r, kH, road_curve->linear_tolerance(), &k_order);
      const double relative_error = (k_order_s_approximation != 0.0)
                                        ? (s_from_p_at_r(p) - k_order_s_approximation) / k_order_s_approximation
                                        : s_from_p_at_r(p);
      EXPECT_LE(relative_error, kTolerance) << fmt::format(
          "Path length estimation with a tolerance of {} "
          "m failed at p = {}, r = {} m, h = {} m with "
          "{} for elevation and {} for superelevation",
          road_curve->linear_tolerance(), p, r, kH, road_curve->elevation(), road_curve->superelevation());
    }
  }
}

// Returns an exhaustive combination of CubicPolynomial instances for testing.
std::vector<CubicPolynomial> GetCubicPolynomials() {
  return {{0.0, 0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {1.0, 1.0, 0.0, 0.0},
          {0.0, 0.0, 1.0, 0.0}, {1.0, 0.0, 1.0, 0.0}, {0.0, 1.0, 1.0, 0.0}, {1.0, 1.0, 1.0, 0.0},
          {0.0, 0.0, 0.0, 1.0}, {1.0, 0.0, 0.0, 1.0}, {0.0, 1.0, 0.0, 1.0}, {1.0, 1.0, 0.0, 1.0},
          {0.0, 0.0, 1.0, 1.0}, {1.0, 0.0, 1.0, 1.0}, {0.0, 1.0, 1.0, 1.0}, {1.0, 1.0, 1.0, 1.0}};
}

// Returns a collection of ArcRoadCurve instances for testing
// that are simple enough for fast analytical computations to be
// accurate.
std::vector<std::shared_ptr<RoadCurve>> GetSimpleLineRoadCurves() {
  const math::Vector2 kStart{1., 1.};
  const math::Vector2 kEnd{10., -8.};
  const double kLinearTolerance{0.01};
  const double kScaleLength{1.};
  const CubicPolynomial zp{0., 0., 0., 0.};

  std::vector<std::shared_ptr<RoadCurve>> road_curves;
  for (const auto& elevation_polynomial : GetCubicPolynomials()) {
    if (elevation_polynomial.order() <= 1) {
      road_curves.push_back(std::make_shared<LineRoadCurve>(kStart, kEnd - kStart, elevation_polynomial, zp,
                                                            kLinearTolerance, kScaleLength,
                                                            ComputationPolicy::kPreferSpeed));
    }
  }
  return road_curves;
}

// Returns a collection of LineRoadCurve instances for testing.
std::vector<std::shared_ptr<RoadCurve>> GetLineRoadCurves() {
  const math::Vector2 kStart{0., 0.};
  const math::Vector2 kEnd{10., -8.};
  const double kLinearTolerance{0.01};
  const double kScaleLength{1.};

  std::vector<std::shared_ptr<RoadCurve>> road_curves;
  for (const auto& elevation_polynomial : GetCubicPolynomials()) {
    for (const auto& superelevation_polynomial : GetCubicPolynomials()) {
      road_curves.push_back(std::make_shared<LineRoadCurve>(kStart, kEnd - kStart, elevation_polynomial,
                                                            superelevation_polynomial, kLinearTolerance, kScaleLength,
                                                            ComputationPolicy::kPreferAccuracy));
    }
  }
  return road_curves;
}

// Returns a collection of ArcRoadCurve instances for testing
// that are simple enough for fast analytical computations to be
// accurate.
std::vector<std::shared_ptr<RoadCurve>> GetSimpleArcRoadCurves() {
  const math::Vector2 kCenter{1., 1.};
  const double kRadius{12.0};
  const double kTheta0{M_PI / 9.};
  const double kDTheta{M_PI / 3.};
  const double kLinearTolerance{0.01};
  const double kScaleLength{1.};
  const CubicPolynomial zp{0., 0., 0., 0.};

  std::vector<std::shared_ptr<RoadCurve>> road_curves;
  for (const auto& elevation_polynomial : GetCubicPolynomials()) {
    if (elevation_polynomial.order() <= 1) {
      road_curves.push_back(std::make_shared<ArcRoadCurve>(kCenter, kRadius, kTheta0, kDTheta, elevation_polynomial, zp,
                                                           kLinearTolerance, kScaleLength,
                                                           ComputationPolicy::kPreferSpeed));
    }
  }
  return road_curves;
}

// Returns a collection of ArcRoadCurve instances for testing.
std::vector<std::shared_ptr<RoadCurve>> GetArcRoadCurves() {
  const math::Vector2 kCenter{0., 0.};
  const double kRadius{10.0};
  const double kTheta0{M_PI / 6.};
  const double kDTheta{M_PI / 2.};
  const double kLinearTolerance{0.01};
  const double kScaleLength{1.};

  std::vector<std::shared_ptr<RoadCurve>> road_curves;
  for (const auto& elevation_polynomial : GetCubicPolynomials()) {
    for (const auto& superelevation_polynomial : GetCubicPolynomials()) {
      road_curves.push_back(std::make_shared<ArcRoadCurve>(kCenter, kRadius, kTheta0, kDTheta, elevation_polynomial,
                                                           superelevation_polynomial, kLinearTolerance, kScaleLength,
                                                           ComputationPolicy::kPreferAccuracy));
    }
  }
  return road_curves;
}

INSTANTIATE_TEST_CASE_P(SimpleAndFastLineRoadCurveAccuracyTest, RoadCurveAccuracyTest,
                        ::testing::ValuesIn(GetSimpleLineRoadCurves()));

INSTANTIATE_TEST_CASE_P(ExhaustiveLineRoadCurveAccuracyTest, RoadCurveAccuracyTest,
                        ::testing::ValuesIn(GetLineRoadCurves()));

INSTANTIATE_TEST_CASE_P(SimpleAndFastArcRoadCurveAccuracyTest, RoadCurveAccuracyTest,
                        ::testing::ValuesIn(GetSimpleArcRoadCurves()));

INSTANTIATE_TEST_CASE_P(ExhaustiveArcRoadCurveAccuracyTest, RoadCurveAccuracyTest,
                        ::testing::ValuesIn(GetArcRoadCurves()));

}  // namespace
}  // namespace multilane
}  // namespace maliput
