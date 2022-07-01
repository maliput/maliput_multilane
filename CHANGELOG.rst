^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package maliput_multilane
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fixes environment hooks. (`#90 <https://github.com/maliput/maliput_multilane/issues/90>`_)
* Fixes dependency on environment variables. (`#89 <https://github.com/maliput/maliput_multilane/issues/89>`_)
* Contributors: Franco Cipollone

0.1.0 (2022-06-16)
------------------
* Updates package.xml.
* Suppresses old-rule-api-related warnings. (`#88 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/88>`_)
* Fixes include folder installation. (`#87 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/87>`_)
* Relies on compile definitions for testing instead of env vars. (`#86 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/86>`_)
* Updates license. (`#85 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/85>`_)
* Uses ament_export_targets. (`#84 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/84>`_)
* Removes dashing support. (`#83 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/83>`_)
* Updates IntersectionBook construction points to include the RoadGeometry pointer. (`#82 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/82>`_)
* Adds BUILD_DOCS flag as opt-in flag (`#81 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/81>`_)
* Matches with change in maliput's phase ring book loader. (`#80 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/80>`_)
* Adds workflow_dispatch to other workflows. (`#79 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/79>`_)
* Adds CI badges (`#78 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/78>`_)
* Replaces push by workflow_dispatch event in gcc build. (`#77 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/77>`_)
* Extends RoadNetworkLoader plugin to include yaml descriptions and OnrampMerge implementation. (`#76 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/76>`_)
* Fixes MALIPUT_PLUGIN_PATH new path. (`#75 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/75>`_)
* Change install path of the maliput plugin. (`#74 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/74>`_)
* Use maliput drake (`#73 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/73>`_)
* Makes multilane to link privately against drake (`#72 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/72>`_)
* Remove eigen_compare_matrices.h (`#71 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/71>`_)
* Use maliput math instead of drake (`#70 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/70>`_)
* Hide drake types part 2 (`#69 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/69>`_)
* Hide drake types part 1 (`#68 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/68>`_)
* Enable doxygen verification. (`#66 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/66>`_)
* Replaces AMENT_CURRENT_PREFIX by COLCON_PREFIX_PATH (`#65 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/65>`_)
* Removes ament_target_dependencies macro. (`#64 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/64>`_)
* Set up linker properly when using clang in CI. (`#63 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/63>`_)
* Remove references to GnuInstallDir vars. (`#62 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/62>`_)
* rosdep update --include-eol-distros (`#61 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/61>`_)
* Enable CI for 20.04 (`#60 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/60>`_)
* Foxy style fix part 2: reorder includes (`#59 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/59>`_)
* pybind11 is not needed in CI (`#58 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/58>`_)
* Foxy include style part 1: use <> for maliput, drake headers (`#57 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/57>`_)
* CI: Removes prereqs install for drake. (`#56 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/56>`_)
* Upgrade to ros-tooling v0.2.1 (`#55 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/55>`_)
* Use maliput_documentation instead of maliput-documentation. (`#54 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/54>`_)
* Use maliput_multilane instead of maliput-multilane (`#53 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/53>`_)
* Switch ament_cmake_doxygen to main branch (`#52 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/52>`_)
* Optimizes scan-build run in CI. (`#51 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/51>`_)
* Add changelog template (`#50 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/50>`_)
* Points to maliput_infrastructure instead of dsim-repos-index (`#49 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/49>`_)
* Trigger PR clang builds on do-clang-test label (`#48 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/48>`_)
* Restores scan-build workflow on label (`#47 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/47>`_)
* Stub implementation of RoadGeometry::do_inertial_to_backend_frame_translation (`#46 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/46>`_)
* Moves disabled workflows to a different folder. (`#44 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/44>`_)
* Refer to a specific clang version and use lld linker. (`#43 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/43>`_)
* Matches with plugin extern c methods refactor. (`#42 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/42>`_)
* Update ros-tooling version in CI. (`#41 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/41>`_)
* Fixes ubsan behavior in CI. (`#39 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/39>`_)
* Fixes plugin test failure when running ubsan. (`#40 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/40>`_)
* Removes Jenkins configuration. (`#37 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/37>`_)
* Append library dirs to plugin test. (`#36 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/36>`_)
* Adds tests for RoadNetworkLoader multilane plugin. (`#35 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/35>`_)
* Implements a maliput RoadNetworkLoader plugin. (`#34 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/34>`_)
* Replaced GeoPosition by InertialPosition (`#33 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/33>`_)
* Unifies cmake install paths. (`#32 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/32>`_)
* Uses namespaces for the targets (`#31 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/31>`_)
* Use maliput::test_utilities and try same branch name in actions (`#30 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/30>`_)
* Eliminate a few warnings induced by switch statements. (`#29 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/29>`_)
* Adds scan_build job to Github Actions. (`#27 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/27>`_)
* Sets ACTIONS_ALLOW_UNSECURE_COMMANDS to true (`#28 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/28>`_)
* Adds clang8, asan, ubsan and tsan to Github Actions. (`#26 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/26>`_)
* Adds a template changelog. (`#24 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/24>`_)
* Updates the package.xml (`#25 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/25>`_)
* Fixes sanitizers variable. (`#22 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/22>`_)
* Use GitHub Actions CI to build and test with gcc (`#21 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/21>`_)
* Migrates drake types because of deprecations. (`#20 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/20>`_)
* Adds scan-build to jenkins configuration. (`#19 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/19>`_)
* Parallelizes CI.
* Static Analyzer: Adds exclusion file.
* Adds Undefined Behavior Sanitizer.
* Adds Address Sanitizer.
* agalbachicar/`#278 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/278>`_ migrate docs to doxygen part 6 (`#13 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/13>`_)
* Pairs clang flags. (`#5 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/5>`_)
* Updates compilation flags for gcc and clang (`#4 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/4>`_)
* Merge pull request `#1 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/1>`_ from ToyotaResearchInstitute/francocipollone/move_multilane_to_a_repo
* Updates package version.
* Updates README
* Updates the LICENSE to include Toyota Research Institute.
* Adapts files to maliput_multilane package.
* Adds missing files to the repository.
* Adds fmt as dependency. (`#283 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/283>`_)
* Implements Quaternion. (`#264 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/264>`_)
* Implements logger to replace spd_log. (`#236 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/236>`_)
* Move eigen_matrix_compare.h file. (One step of `#260 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/260>`_)
* Matrix library implementation. (`#237 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/237>`_)
* Replaces calls to drake::Quaternion<T> by maliput::math::Quaternion (`#256 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/256>`_)
* Vector library implementation. (`#237 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/237>`_)
* Replaces drake::VectorN<double> by maliput::math::VectorN. (`#251 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/251>`_)
* Migrates drake_copyable.h. (`#240 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/240>`_)
* Migrates drake::unused(). (`#241 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/241>`_)
* Adjust to a new drake version.
* Upgrade to c++17.
* Group segments within 2x2_intersection (`#217 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/217>`_)
* Build documentation by default. (`#206 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/206>`_)
* Use ament_cmake_doxygen to generate C++ documentation.  (`#165 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/165>`_)
* Modifies return value of Lane::ToLanePosition() (`#163 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/163>`_)
* Modifies ToRoadPosition to return a RoadPositionResult. (`#160 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/160>`_)
* Moves Lane::driveable_bounds() to Lane::segment_bounds(). (`#154 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/154>`_)
* Add cmake clang format (`#113 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/113>`_)
* Add auto clang formatting check to colcon test (`#98 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/98>`_)
* Reformat to obey TRI style (`#87 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/87>`_)
* Logger support in maliput (`#89 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/89>`_)
* Add gmock to tests requiring gmock
* Migrates DRAKE_THROW_UNLESS to MALIPUT_THROW_UNLESS (`#74 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/74>`_)
* Adds to RightOfWayRule a map of TrafficLight Ids --> BulbGroup Ids (`#79 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/79>`_)
* Replaces DRAKE\_*-aborts by MALIPUT\_* (`#73 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/73>`_)
* Adapt packages to withstand tarball installation (`#61 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/61>`_)
* Adds RoadGeometry::FindRoadPositions() (`#58 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/58>`_)
* Removed redundant maliput dir
* Create maliput ament packages
* Adds TrafficLightBook loader
* Adds Maliput Phase Ring Book Loader (`#11021 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/11021>`_)
* Generalizes RightOfWayRules Loader (`#10977 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/10977>`_)
* Adds Maliput RightOfWayRule Loader (`#10949 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/10949>`_)
* Adds unit test to 2x2_intersection_test.cc (`#10891 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/10891>`_)
* Adds a unit test for 2x2_intersection.yaml (`#10841 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/10841>`_)
* Cleans up maliput/multilane/BUILD.bazel (`#10876 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/10876>`_)
* Adds 2x2_intersection.yaml to Maliput multilane (`#10834 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/10834>`_)
* Fixes grammar in maliput/multilane/loader.h (`#10815 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/10815>`_)
* Deprecate and remove all uses of DRAKE_ABORT_MSG (`#10781 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/10781>`_)
* Deprecate and remove all uses of DRAKE_ABORT (`#10545 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/10545>`_)
* In lane_data.h, quaternion.toRotationMatrix() to new RotationMatrix constructor.
* Improves explicit theta_dot treatment in Multilane's Builder.
* Miscellaneous documentation fixes
* Documentation fixes
* Make various lists in Doxygen both Markdown and reST friendly
* Clean up exception specification in doxygen
* Add some missing \note doxygen tags
* Express characteristic scale length concept in api::RoadGeometry (`#9306 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/9306>`_)
* [multilane] Fixes ToRoadPosition to correctly use `r` coordinate (`#9464 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/9464>`_)
* Fixes broken Multilane loader tests.
* Adds multi-lane road tests to Multilane Loader test suite (`#9302 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/9302>`_)
* [multilane] Group-related interfaces to allow loader testing (`#9278 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/9278>`_)
* Fix tolerance usage in Endpoint comparisons.
* Adds `multilane` YAML format documentation. (`#9208 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/9208>`_)
* Renames RoadCurve's p_scale to l_max. (`#9332 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/9332>`_)
* Merge pull request `#9155 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/9155>`_ from ekumenlabs/Issue/Multilane_Optimized_RoadCurve_Computations
* Disable multilane_loader_test under ubsan and the use of sanitizer blacklists in general
* Issue/`#8530 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/8530>`_ replace string concat by fmt (`#9093 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/9093>`_)
* [Multilane] Loader to support lane-to-lane connections. (`#9090 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/9090>`_)
* Adds lane-to-lane Builder::Connect methods. (`#8973 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/8973>`_)
* [Multilane] Adds continuity constraints into the loader (`#8676 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/8676>`_)
* Continue deprecating rpy2rotmat in favor of existing and newly created methods in RollPitchYaw class  (`#8969 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/8969>`_).
* Add an IdIndex interface to maliput::api::RoadGeometry.
* Propagates computational settings from Loader to RoadCurve.
* Add num_shards=10 for multilane_lanes_test
* Use shards for multilane_road_curve_accuracy_test.
* Added arc length approximation support to Multilane's RoadCurve class.
* [Maliput] Utility OBJ Parser (`#8679 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/8679>`_)
* Multilane's Builder API refactor in favor of fluent API (`#8302 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/8302>`_)
* Add RotationMatrix constructor that takes RollPitchYaw argument and change #include roll_pitch_yaw.h to #include rotation_matrix.h
* Re-apply "Add drake_cc_package_library and library_lint"
* Remove test size when it matches default of small
* Blacklist bits/stl_tree.h for UBSan
* Revert "Add drake_cc_package_library and library_lint"
* Add drake_cc_package_library and library_lint
* Multilane restore loader gmock tests (`#8565 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/8565>`_)
* Revert "Modifies multilane's Loader tests using gmock. (`#8071 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/8071>`_)"
* Modifies multilane's Loader tests using gmock. (`#8071 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/8071>`_)
* Fix Wshadow warnings from GCC
* Add some const hints to improve readability
* Fixes multilane's ToRoadPosition `#8045 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/8045>`_ (`#8212 <https://github.com/ToyotaResearchInstitute/maliput_multilane/issues/8212>`_)
* Add drake_py_unittest helper
* Fix //drake label names in automotive/maliput/multilane/test_utilities
* Fix //drake label names in automotive/maliput/multilane
* Unifies unnamed namespaces for rndf and multilane tests.
* Multilane's Loader, part I.
* Run tools/dev/6996-move
* Initial commit
* Contributors: Agustin Alba Chicar, Andrés Valenzuela, Brian, Chien-Liang Fok, Daniel Stonier, Drake Refactor Bot, Duy-Nguyen Ta, Franco, Franco Cipollone, Geoffrey Biggs, Jamie Snape, Jeremy Nimmer, John, John Shepherd, Matt Marjanovic, Matt Marjanović, Michel Hidalgo, Mitiguy, Mmanu Chaturvedi, Steve Peters, mitiguy
