find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)

macro(add_dependencies_to_test target)
    if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
          ${PROJECT_SOURCE_DIR}/test
      )

      set(MULTILANE_RESOURCE_ROOT ${PROJECT_SOURCE_DIR}/resources/)
      target_compile_definitions(${target}
        PRIVATE
          DEF_MULTILANE_RESOURCE_ROOT="${MULTILANE_RESOURCE_ROOT}"
      )

      target_link_libraries(${target}
          maliput::api
          maliput::common
          maliput::drake_common
          maliput::math
          maliput_multilane::maliput_multilane
          maliput_multilane::test_utilities
      )

    endif()
endmacro()

add_subdirectory(maliput_multilane)
add_subdirectory(maliput_multilane_test_utilities)
add_subdirectory(plugin)
