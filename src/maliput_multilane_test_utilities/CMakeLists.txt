if(BUILD_TESTING)

find_package(ament_cmake_gmock REQUIRED)
ament_find_gmock()

set(TEST_UTILS_SOURCES
  fixtures.cc
  multilane_types_compare.cc)

add_library(test_utilities ${TEST_UTILS_SOURCES})

add_library(maliput_multilane::test_utilities ALIAS test_utilities)

set_target_properties(test_utilities
  PROPERTIES
    OUTPUT_NAME maliput_multilane_test_utilities
)

target_include_directories(
  test_utilities
  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
  ${GMOCK_INCLUDE_DIRS}
  )

target_link_libraries(test_utilities
  PUBLIC
    maliput::api
    maliput::common
    maliput::test_utilities
    maliput::math
    maliput_multilane
  PRIVATE
    maliput_drake::analysis
    maliput_drake::common
    maliput_drake::framework
    maliput_drake::math
    maliput_drake::trajectories
)

install(
  TARGETS test_utilities
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

endif()
