// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include <yaml-cpp/yaml.h>

#include "maliput_multilane/builder.h"
#include "maliput_multilane/loader.h"
#include "maliput_multilane/params.h"

namespace maliput {
namespace multilane {
namespace {

// @returns true if `node` contains a subnode named "RoadRulebook".
bool IsRoadRulebookNodeDefined(const YAML::Node& node) {
  const YAML::Node& rulebook_node = node["RoadRulebook"];
  return rulebook_node.IsDefined();
}

// @returns true if `node` contains a subnode named "TrafficLights".
bool IsTrafficLightsNodeDefined(const YAML::Node& node) {
  const YAML::Node& rulebook_node = node["TrafficLights"];
  return rulebook_node.IsDefined();
}

// @returns true if `node` contains a subnode named "PhaseRings".
bool IsPhaseRingsNodeDefined(const YAML::Node& node) {
  const YAML::Node& rulebook_node = node["PhaseRings"];
  return rulebook_node.IsDefined();
}

// @returns true if `node` contains a subnode named "Intersections".
bool IsIntersectionsNodeDefined(const YAML::Node& node) {
  const YAML::Node& rulebook_node = node["Intersections"];
  return rulebook_node.IsDefined();
}

}  // namespace

RoadNetworkConfiguration RoadNetworkConfiguration::FromMap(
    const std::map<std::string, std::string>& road_network_configuration) {
  RoadNetworkConfiguration config;
  auto it = road_network_configuration.find(params::kYamlFile);
  if (it != road_network_configuration.end()) {
    config.yaml_file = it->second;
  }
  it = road_network_configuration.find(params::kYamlDescription);
  if (it != road_network_configuration.end()) {
    config.yaml_description = it->second;
  }
  return config;
}

std::map<std::string, std::string> RoadNetworkConfiguration::ToStringMap() const {
  return {{params::kYamlFile, yaml_file}, {params::kYamlDescription, yaml_description}};
}

std::unique_ptr<api::RoadNetwork> BuildRoadNetwork(const RoadNetworkConfiguration& road_network_configuration) {
  maliput::log()->debug("Building multilane RoadNetwork.");

  if (road_network_configuration.yaml_file.empty() && road_network_configuration.yaml_description.empty()) {
    MALIPUT_ABORT_MESSAGE("Both yaml_file and yaml_description cannot be empty.");
  }
  auto yaml_root_node = !road_network_configuration.yaml_file.empty()
                            ? YAML::LoadFile(road_network_configuration.yaml_file)
                            : YAML::Load(road_network_configuration.yaml_description);
  auto rg = !road_network_configuration.yaml_file.empty()
                ? LoadFile(BuilderFactory(), road_network_configuration.yaml_file)
                : Load(BuilderFactory(), road_network_configuration.yaml_description);

  auto rulebook = IsRoadRulebookNodeDefined(yaml_root_node)
                      ? (!road_network_configuration.yaml_file.empty()
                             ? LoadRoadRulebookFromFile(rg.get(), road_network_configuration.yaml_file)
                             : LoadRoadRulebook(rg.get(), road_network_configuration.yaml_description))
                      : std::make_unique<ManualRulebook>();
  auto traffic_light_book = IsTrafficLightsNodeDefined(yaml_root_node)
                                ? (!road_network_configuration.yaml_file.empty()
                                       ? LoadTrafficLightBookFromFile(road_network_configuration.yaml_file)
                                       : LoadTrafficLightBook(road_network_configuration.yaml_description))
                                : std::make_unique<TrafficLightBook>();
  auto phase_ring_book = IsPhaseRingsNodeDefined(yaml_root_node)
                             ? (!road_network_configuration.yaml_file.empty()
                                    ? LoadPhaseRingBookFromFileOldRules(rulebook.get(), traffic_light_book.get(),
                                                                        road_network_configuration.yaml_file)
                                    : LoadPhaseRingBookOldRules(rulebook.get(), traffic_light_book.get(),
                                                                road_network_configuration.yaml_description))
                             : std::make_unique<ManualPhaseRingBook>();
  std::unique_ptr<ManualPhaseProvider> phase_provider = std::make_unique<ManualPhaseProvider>();
  auto intersection_book = IsIntersectionsNodeDefined(yaml_root_node)
                               ? (!road_network_configuration.yaml_file.empty()
                                      ? LoadIntersectionBookFromFile(road_network_configuration.yaml_file, *rulebook,
                                                                     *phase_ring_book, rg.get(), phase_provider.get())
                                      : LoadIntersectionBook(road_network_configuration.yaml_file, *rulebook,
                                                             *phase_ring_book, rg.get(), phase_provider.get()))
                               : std::make_unique<IntersectionBook>(rg.get());
  std::unique_ptr<api::rules::RuleRegistry> rule_registry = std::make_unique<api::rules::RuleRegistry>();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::unique_ptr<ManualRightOfWayRuleStateProvider> right_of_way_rule_state_provider =
      std::make_unique<ManualRightOfWayRuleStateProvider>();
#pragma GCC diagnostic pop
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

std::unique_ptr<api::RoadNetwork> BuildOnRampMergeRoadNetwork(
    const MultilaneRoadCharacteristics& road_characteristics) {
  maliput::log()->debug("Building multilane onramp merge RoadNetwork.");
  auto rg = maliput::multilane::MultilaneOnrampMerge(road_characteristics).BuildOnramp();
  auto rulebook = std::make_unique<ManualRulebook>();
  auto traffic_light_book = std::make_unique<TrafficLightBook>();
  auto phase_ring_book = std::make_unique<ManualPhaseRingBook>();
  auto result = std::make_unique<ManualPhaseRingBook>();
  auto phase_provider = std::make_unique<ManualPhaseProvider>();
  auto intersection_book = std::make_unique<IntersectionBook>(rg.get());
  std::unique_ptr<api::rules::RuleRegistry> rule_registry = std::make_unique<api::rules::RuleRegistry>();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::unique_ptr<ManualRightOfWayRuleStateProvider> right_of_way_rule_state_provider =
      std::make_unique<ManualRightOfWayRuleStateProvider>();
#pragma GCC diagnostic pop
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
