#pragma once

#include <memory>
#include <string>

#include "maliput_multilane/builder.h"

namespace maliput {

namespace api {
class RoadGeometry;
}

namespace multilane {

/// Loads the `input` string as a maliput_multilane_builder document using the
/// provided `builder_factory`. See @ref loader.h documentation for further details.
///
/// Application code must use a BuilderFactory reference. It is provided so that
/// the @ref maliput::multilane::Builder "Builder" to be created can be
/// mocked and code can be tested.
std::unique_ptr<const api::RoadGeometry> Load(const BuilderFactoryBase& builder_factory, const std::string& input);

/// Loads the named file as a maliput_multilane_builder document using the
/// provided `builder_factory`. See @ref loader.h documentation for further details.
///
/// Application code must use a BuilderFactory reference. It is provided so that
/// the @ref maliput::multilane::Builder "Builder" to be created can be
/// mocked and code can be tested.
std::unique_ptr<const api::RoadGeometry> LoadFile(const BuilderFactoryBase& builder_factory,
                                                  const std::string& filename);

}  // namespace multilane
}  // namespace maliput
