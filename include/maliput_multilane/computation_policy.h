#pragma once

namespace maliput {
namespace multilane {

/// A policy to guide all computations in a RoadCurve, in terms of
/// speed and accuracy.
enum class ComputationPolicy {
  kPreferAccuracy,  ///< Always prefer accurate results,
                    ///  even if at a slower pace (e.g. using
                    ///  expensive numerical approximations).
  kPreferSpeed      ///< Always prefer fast computations,
                    ///  even if not accurate (e.g. using
                    ///  approximated analytical expressions).
};

}  // namespace multilane
}  // namespace maliput
