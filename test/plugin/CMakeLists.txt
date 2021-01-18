find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(road_network_plugin road_network_plugin_test.cc)

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
