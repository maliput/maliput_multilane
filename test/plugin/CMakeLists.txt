find_package(ament_cmake_gtest REQUIRED)

# The following test uses the maliput::plugin::MaliputPluginManager entity to load all the availables plugins.
# Given that the LD_LIBRARY_PATH environment variable isn't correctly set up until setup.bash is sourced,
# and to avoid sourcing that file just to run one test we should append the paths.
# This allow to any other backend plugin to find its correspondant backend core library.
ament_add_gtest(road_network_plugin road_network_plugin_test.cc
    APPEND_LIBRARY_DIRS
      ${CMAKE_INSTALL_PREFIX}/../maliput_dragway/lib
      ${CMAKE_INSTALL_PREFIX}/../maliput_malidrive/lib
)

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
