// Copyright 2019 Carlos San Vicente
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

#include <memory>
#include <utility>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "pendulum_driver/pendulum_driver_node.hpp"
#include "pendulum_controller/pendulum_controller_node.hpp"
#include "pendulum_tools/process_settings.hpp"
#include "pendulum_tools/lifecycle_autostart.hpp"

int main(int argc, char * argv[])
{
  std::vector<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>> nodes;
  pendulum::tools::ProcessSettings settings;
  if (!settings.init(argc, argv)) {
    return EXIT_FAILURE;
  }

  int32_t ret = 0;

  try {
    // configure process real-time settings
    if (settings.configure_child_threads) {
      // process child threads created by ROS nodes will inherit the settings
      settings.configure_process();
    }

    rclcpp::init(argc, argv);

    // Create a static executor
    rclcpp::executors::StaticSingleThreadedExecutor exec;

    // Create pendulum controller node
    if (settings.controller_enable) {
      RCLCPP_INFO(rclcpp::get_logger("pendulum_demo"), "Pendulum controller enabled");
      using pendulum::pendulum_controller::PendulumControllerNode;
      const auto controller_node_ptr =
          std::make_shared<PendulumControllerNode>("pendulum_controller");

      exec.add_node(controller_node_ptr->get_node_base_interface());
      nodes.push_back(std::move(controller_node_ptr));
    } else {
      RCLCPP_INFO(rclcpp::get_logger("pendulum_demo"), "Pendulum controller disabled");
    }

    // Create pendulum simulation
    if (settings.driver_enable) {
      RCLCPP_INFO(rclcpp::get_logger("pendulum_demo"), "Pendulum driver enabled");
      using pendulum::pendulum_driver::PendulumDriverNode;
      const auto driver_node_ptr = std::make_shared<PendulumDriverNode>("pendulum_driver");

      exec.add_node(driver_node_ptr->get_node_base_interface());
      nodes.push_back(std::move(driver_node_ptr));
    } else {
      RCLCPP_INFO(rclcpp::get_logger("pendulum_demo"), "Pendulum driver disabled");
    }

    // configure process real-time settings
    if (!settings.configure_child_threads) {
      // process child threads created by ROS nodes will NOT inherit the settings
      settings.configure_process();
    }

    if (settings.auto_start_nodes) {
      for (auto node = nodes.begin(); node < nodes.end(); node++) {
        pendulum::tools::autostart(**node);
      }
    }

    exec.spin();
    rclcpp::shutdown();
  } catch (const std::exception & e) {
    RCLCPP_INFO(rclcpp::get_logger("pendulum_demo"), e.what());
    ret = 2;
  } catch (...) {
    RCLCPP_INFO(
      rclcpp::get_logger("pendulum_demo"), "Unknown exception caught. "
      "Exiting...");
    ret = -1;
  }
  return ret;
}
