#pragma once

#include "maliput_multilane/connection.h"
#include "maliput_multilane/road_curve.h"

namespace maliput {
namespace multilane {

/// Creates a RoadCurve that describes the reference curve of @p connection.
/// @param connection The Connection to create a RoadCurve from.
/// @return A RoadCurve (LineRoadCurve or ArcRoadCurve based on
///         @p connection.type()) that describes the reference curve of the
///         @p connection.
std::unique_ptr<RoadCurve> MakeRoadCurveFor(const Connection& connection);

}  // namespace multilane
}  // namespace maliput
