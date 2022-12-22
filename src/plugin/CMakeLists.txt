##############################################################################
# Sources
##############################################################################

### Maliput Multilane Road Network Plugin.
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
    $<INSTALL_INTERFACE:include>
)

### Maliput Multilane On-Ramp Road Network Plugin.
add_library(on_ramp_road_network on_ramp_road_network.cc)
add_library(maliput_multilane::on_ramp_road_network ALIAS on_ramp_road_network)

set_target_properties(on_ramp_road_network
  PROPERTIES
    OUTPUT_NAME maliput_multilane_on_ramp_road_network
)

target_link_libraries(on_ramp_road_network
  maliput::api
  maliput::common
  maliput_multilane::maliput_multilane
)

target_include_directories(on_ramp_road_network
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

##############################################################################
# Export
##############################################################################

# Using a different location as this target is a dynamic library
# which will be loaded in runtime as a maliput plugin.
set(PLUGIN_INSTALL_DIR
    lib/maliput_plugins
)

install(
  TARGETS road_network on_ramp_road_network
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION ${PLUGIN_INSTALL_DIR}
  RUNTIME DESTINATION bin
)
