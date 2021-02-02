find_package(ament_cmake_gtest REQUIRED)

set (LIBRARY_DIRS ${CMAKE_INSTALL_PREFIX}/../maliput_dragway/lib ${CMAKE_INSTALL_PREFIX}/../maliput_malidrive/lib)
# The following test uses the maliput::plugin::MaliputPluginManager entity to load all the availables plugins.
# Given that the LD_LIBRARY_PATH environment variable isn't correctly set up until setup.bash is sourced,
# and to avoid sourcing that file just to run one test we should append the paths.
# This allow to any other backend plugin to find its correspondant backend core library.
ament_add_gtest(road_network_plugin road_network_plugin_test.cc APPEND_LIBRARY_DIRS ${LIBRARY_DIRS})

macro(add_dependencies_to_test target)
    if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
      )

      ament_target_dependencies(${target}
        "maliput"
      )

      target_link_libraries(${target}
          maliput_multilane::maliput_multilane
          maliput::common
          maliput::plugin
      )
    endif()
endmacro()

add_dependencies_to_test(road_network_plugin)
