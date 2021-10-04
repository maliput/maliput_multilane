##############################################################################
# Sources
##############################################################################

set(MALIPUT_MULTILANE_SOURCES
  arc_road_curve.cc
  branch_point.cc
  builder.cc
  connection.cc
  cubic_polynomial.cc
  junction.cc
  lane.cc
  line_road_curve.cc
  loader.cc
  make_road_curve_for_connection.cc
  multilane_onramp_merge.cc
  road_curve.cc
  road_geometry.cc
  road_network_builder.cc
  segment.cc)

add_library(maliput_multilane ${MALIPUT_MULTILANE_SOURCES})

add_library(maliput_multilane::maliput_multilane ALIAS maliput_multilane)

set_target_properties(maliput_multilane
  PROPERTIES
    OUTPUT_NAME maliput_multilane
)

target_include_directories(maliput_multilane
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(maliput_multilane
  PUBLIC
    maliput::api
    maliput::base
    maliput::common
    maliput::math
  PRIVATE
    drake::drake
    yaml-cpp
)

##############################################################################
# Export
##############################################################################

install(
  TARGETS maliput_multilane
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(maliput_multilane)
