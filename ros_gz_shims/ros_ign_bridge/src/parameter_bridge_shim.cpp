// Shim to redirect "ros_ign_bridge parameter_bridge" call to "ros_gz_bridge parameter_bridge"

#include <string>
#include <sstream>
#include <iostream>
#include <stdlib.h>

#include <ament_index_cpp/get_package_prefix.hpp>


int main(int argc, char * argv[])
{
  std::stringstream cli_call;

  cli_call << ament_index_cpp::get_package_prefix("ros_gz_bridge")
           << "/lib/ros_gz_bridge/parameter_bridge";

  if (argc > 1)
  {
    for (int i = 1; i < argc; i++)
      cli_call << " " << argv[i];
  }

  std::cerr << "[ros_ign_bridge] is deprecated! "
            << "Redirecting to use [ros_gz_bridge] instead!"
            << std::endl << std::endl;
  system(cli_call.str().c_str());

  return 0;
}
