// Copyright 2026 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <memory>
#include <utility>

#include "handler_base.hpp"

namespace
{
class ResourceHandler : public ros_gz_sim::gz_simulation_interfaces::HandlerBase
{
public:
  explicit ResourceHandler(std::shared_ptr<int> resource)
  : HandlerBase(nullptr, nullptr), resource_(std::move(resource))
  {
  }

private:
  std::shared_ptr<int> resource_;
};
}  // namespace

TEST(HandlerLifetime, ReleasesDerivedResourcesThroughBaseOwner)
{
  auto resource = std::make_shared<int>(42);
  std::weak_ptr<int> lifetime = resource;
  std::unique_ptr<ros_gz_sim::gz_simulation_interfaces::HandlerBase> handler =
    std::make_unique<ResourceHandler>(resource);
  resource.reset();
  ASSERT_FALSE(lifetime.expired());

  handler.reset();
  EXPECT_TRUE(lifetime.expired());
}
