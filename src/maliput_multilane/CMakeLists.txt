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
  multilane_onramp_merge.cc
  road_curve.cc
  road_geometry.cc
  segment.cc)

add_library(maliput_multilane ${MALIPUT_MULTILANE_SOURCES})

target_include_directories(maliput_multilane
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

ament_target_dependencies(maliput_multilane
  "maliput"
  "drake"
)

target_link_libraries(maliput_multilane
  drake::drake
  maliput::api
  maliput::common
  maliput::math
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
    RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

ament_export_libraries(maliput_multilane)
