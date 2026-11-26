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

#ifndef RGBA_HPP_
#define RGBA_HPP_

#include <bit>
#include <cstdint>

namespace ros_gz_point_cloud
{

/// \brief Colour of a point.
struct Rgb
{
  /// \brief Red.
  std::uint8_t r;

  /// \brief Green.
  std::uint8_t g;

  /// \brief Blue.
  std::uint8_t b;
};

/// \brief Decode the colour that Gazebo packs into the 4th float of each
/// point of an RGBD camera's point cloud.
///
/// Gazebo packs it as `R << 24 | G << 16 | B << 8 | A`, the same as
/// `DecodeRGBAFromFloat` in gz-sensors. Decoding with shifts instead of
/// reading the bytes works on any endianness.
/// \param[in] _rgba Packed colour.
/// \return Red, green and blue of the packed colour.
inline Rgb decodePackedRgba(float _rgba)
{
  const auto rgba = std::bit_cast<std::uint32_t>(_rgba);
  return {
    static_cast<std::uint8_t>(rgba >> 24 & 0xFF),
    static_cast<std::uint8_t>(rgba >> 16 & 0xFF),
    static_cast<std::uint8_t>(rgba >> 8 & 0xFF)};
}

}  // namespace ros_gz_point_cloud

#endif  // RGBA_HPP_
