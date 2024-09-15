// Copyright 2024 James Pace
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
#include <fmt/core.h>

#include <j7s-rostrace/RosTraceNode.hpp>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    auto node = std::make_shared<RosTraceNode>();
    exec.add_node(node);
    // Spin once.
    exec.spin_some();

    const auto topics_and_types = node->getTopicNamesAndTypes();
    const auto node_names = node->getNodeNames();
    if (not topics_and_types or not node_names)
    {
        fmt::print("I never got anything.\n");
        return -1;
    }
    for (auto [topic, types] : topics_and_types.value())
    {
        fmt::print("topic: {} -> type: {}\n", topic, types[0]);
    }
    for (auto node : node_names.value())
    {
        fmt::print("Node: {}\n", node);
    }
    rclcpp::shutdown();
    return 0;
}
