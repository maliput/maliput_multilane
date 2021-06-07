// Copyright 2021 Toyota Research Institute
#include "maliput_multilane/road_network_builder.h"

#include <maliput/base/intersection_book.h>
#include <maliput/base/intersection_book_loader.h>
#include <maliput/base/manual_discrete_value_rule_state_provider.h>
#include <maliput/base/manual_phase_provider.h>
#include <maliput/base/manual_phase_ring_book.h>
#include <maliput/base/manual_range_value_rule_state_provider.h>
#include <maliput/base/manual_right_of_way_rule_state_provider.h>
#include <maliput/base/manual_rulebook.h>
#include <maliput/base/phase_ring_book_loader.h>
#include <maliput/base/road_rulebook_loader.h>
#include <maliput/base/traffic_light_book.h>
#include <maliput/base/traffic_light_book_loader.h>
#include <maliput/common/logger.h>
#include <maliput/common/maliput_abort.h>

#include "maliput_multilane/builder.h"
#include "maliput_multilane/loader.h"

#include "yaml-cpp/yaml.h"

namespace maliput {
namespace multilane {

std::unique_ptr<const api::RoadNetwork> BuildRoadNetwork(const RoadNetworkConfiguration& build_properties) {
  maliput::log()->debug("Building multilane RoadNetwork.");
  if (build_properties.yaml_file.empty()) {
    MALIPUT_ABORT_MESSAGE("yaml_file cannot be empty.");
  }
  auto rg = LoadFile(BuilderFactory(), build_properties.yaml_file);
  auto rulebook = LoadRoadRulebookFromFile(rg.get(), build_properties.yaml_file);
  auto traffic_light_book = LoadTrafficLightBookFromFile(build_properties.yaml_file);
  auto phase_ring_book =
      LoadPhaseRingBookFromFile(rulebook.get(), traffic_light_book.get(), build_properties.yaml_file);
  std::unique_ptr<ManualPhaseProvider> phase_provider = std::make_unique<ManualPhaseProvider>();
  auto intersection_book =
      LoadIntersectionBookFromFile(build_properties.yaml_file, *rulebook, *phase_ring_book, phase_provider.get());
  std::unique_ptr<api::rules::RuleRegistry> rule_registry = std::make_unique<api::rules::RuleRegistry>();

  std::unique_ptr<ManualRightOfWayRuleStateProvider> right_of_way_rule_state_provider =
      std::make_unique<ManualRightOfWayRuleStateProvider>();
  std::unique_ptr<ManualDiscreteValueRuleStateProvider> discrete_value_rule_state_provider =
      std::make_unique<ManualDiscreteValueRuleStateProvider>(rulebook.get());
  std::unique_ptr<ManualRangeValueRuleStateProvider> range_value_rule_state_provider =
      std::make_unique<ManualRangeValueRuleStateProvider>(rulebook.get());
  return std::make_unique<api::RoadNetwork>(std::move(rg), std::move(rulebook), std::move(traffic_light_book),
                                            std::move(intersection_book), std::move(phase_ring_book),
                                            std::move(right_of_way_rule_state_provider), std::move(phase_provider),
                                            std::move(rule_registry), std::move(discrete_value_rule_state_provider),
                                            std::move(range_value_rule_state_provider));
}

}  // namespace multilane
}  // namespace maliput
