find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(road_network_plugin road_network_plugin_test.cc)
ament_add_gtest(on_ramp_road_network_plugin on_ramp_road_network_plugin_test.cc)

macro(add_dependencies_to_test target)
    if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
      )

      # To avoid a false positive when running ubsan the symbols must be exported
      # See https://stackoverflow.com/questions/57361776/use-ubsan-with-dynamically-loaded-shared-libraries
      set_target_properties(${target}
        PROPERTIES
          ENABLE_EXPORTS ON
      )

      set(MULTILANE_RESOURCE_ROOT ${PROJECT_SOURCE_DIR}/resources/)
      set(ROAD_NETWORK_PLUGIN ${CMAKE_INSTALL_PREFIX}/lib/maliput_plugins/)
      target_compile_definitions(${target}
        PRIVATE
          DEF_MULTILANE_RESOURCE_ROOT="${MULTILANE_RESOURCE_ROOT}"
          DEF_ROAD_NETWORK_PLUGIN="${ROAD_NETWORK_PLUGIN}"
      )

      target_link_libraries(${target}
          maliput_multilane::maliput_multilane
          maliput::common
          maliput::plugin
          # maliput_drake is required because we include maliput_multilane/road_geometry.h to verify
          # the type of the loaded plugin. That requires other types to be imported as well which
          # boils down up to road_curve.h.
          maliput_drake::analysis
          maliput_drake::common
          maliput_drake::framework
          maliput_drake::math
          maliput_drake::trajectories
      )
    endif()
endmacro()

add_dependencies_to_test(road_network_plugin)
add_dependencies_to_test(on_ramp_road_network_plugin)
