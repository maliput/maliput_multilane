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
#include "maliput_multilane_test_utilities/fixtures.h"

#include <limits>
#include <string>

#include <maliput/api/lane.h>
#include <maliput/common/filesystem.h>

#include "maliput_multilane/builder.h"
#include "maliput_multilane/loader.h"

using maliput::api::LaneId;
using maliput::multilane::BuilderFactory;
using maliput::multilane::LoadFile;

namespace maliput {
namespace multilane {

constexpr char MULTILANE_RESOURCE_VAR[] = "MULTILANE_RESOURCE_ROOT";

BranchAndMergeBasedTest::BranchAndMergeBasedTest()
    : road_geometry_(LoadFile(BuilderFactory(), std::string(DEF_MULTILANE_RESOURCE_ROOT) + "/branch_and_merge.yaml")),
      index_(road_geometry_->ById()),
      total_length_(index_.GetLane(LaneId("l:1.1_0"))->length() + index_.GetLane(LaneId("l:1.2_0"))->length() +
                    index_.GetLane(LaneId("l:1.3_0"))->length()) {}

LoopBasedTest::LoopBasedTest()
    : road_geometry_(LoadFile(BuilderFactory(), std::string(DEF_MULTILANE_RESOURCE_ROOT) + "/loop.yaml")),
      index_(road_geometry_->ById()) {}

MultiBranchBasedTest::MultiBranchBasedTest()
    : road_geometry_(LoadFile(BuilderFactory(), std::string(DEF_MULTILANE_RESOURCE_ROOT) + "/multi_branch.yaml")),
      index_(road_geometry_->ById()) {}

}  // namespace multilane
}  // namespace maliput
