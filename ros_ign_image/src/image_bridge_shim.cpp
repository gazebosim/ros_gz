// Copyright 2022 Open Source Robotics Foundation, Inc.
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

// Shim to redirect "ros_ign_image image_bridge" call to "ros_gz_image image_bridge"

#include <sstream>
#include <iostream>
#include <stdlib.h>

#include <ament_index_cpp/get_package_prefix.hpp>


int main(int argc, char * argv[])
{
  std::stringstream cli_call;

  cli_call << ament_index_cpp::get_package_prefix("ros_gz_image")
           << "/lib/ros_gz_image/image_bridge";

  if (argc > 1)
  {
    for (int i = 1; i < argc; i++)
      cli_call << " " << argv[i];
  }

  std::cerr << "[ros_ign_bridge] is deprecated! "
            << "Redirecting to use [ros_gz_image] instead!"
            << std::endl << std::endl;
  system(cli_call.str().c_str());

  return 0;
}
