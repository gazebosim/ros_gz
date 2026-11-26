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

#include <bit>
#include <cstdint>

#include "rgba.hpp"

using ros_gz_point_cloud::decodePackedRgba;

namespace
{
/// \brief Pack a colour the way Gazebo does: R << 24 | G << 16 | B << 8 | A.
float packRgba(std::uint32_t _r, std::uint32_t _g, std::uint32_t _b, std::uint32_t _a)
{
  return std::bit_cast<float>(_r << 24 | _g << 16 | _b << 8 | _a);
}
}  // namespace

TEST(DecodePackedRgba, DistinctChannels)
{
  const auto rgb = decodePackedRgba(packRgba(0x11, 0x22, 0x33, 0x44));
  EXPECT_EQ(0x11, rgb.r);
  EXPECT_EQ(0x22, rgb.g);
  EXPECT_EQ(0x33, rgb.b);
}

TEST(DecodePackedRgba, PrimaryColors)
{
  const auto red = decodePackedRgba(packRgba(255, 0, 0, 255));
  EXPECT_EQ(255, red.r);
  EXPECT_EQ(0, red.g);
  EXPECT_EQ(0, red.b);

  const auto green = decodePackedRgba(packRgba(0, 255, 0, 255));
  EXPECT_EQ(0, green.r);
  EXPECT_EQ(255, green.g);
  EXPECT_EQ(0, green.b);

  const auto blue = decodePackedRgba(packRgba(0, 0, 255, 255));
  EXPECT_EQ(0, blue.r);
  EXPECT_EQ(0, blue.g);
  EXPECT_EQ(255, blue.b);
}

// Alpha doesn't leak into the colour.
TEST(DecodePackedRgba, AlphaIsIgnored)
{
  const auto opaque = decodePackedRgba(packRgba(10, 20, 30, 255));
  const auto transparent = decodePackedRgba(packRgba(10, 20, 30, 0));
  EXPECT_EQ(opaque.r, transparent.r);
  EXPECT_EQ(opaque.g, transparent.g);
  EXPECT_EQ(opaque.b, transparent.b);
  EXPECT_EQ(10, opaque.r);
}
