##############################################################################
# Sources
##############################################################################

add_library(road_network road_network.cc)
add_library(maliput_multilane::road_network ALIAS road_network)

set_target_properties(road_network
  PROPERTIES
    OUTPUT_NAME maliput_multilane_road_network
)

target_link_libraries(road_network
  maliput::api
  maliput::common
  maliput_multilane::maliput_multilane
)

target_include_directories(road_network
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

##############################################################################
# Export
##############################################################################

install(
  TARGETS road_network
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION "${CMAKE_INSTALL_PREFIX}/../maliput/lib/maliput/plugins"
)
