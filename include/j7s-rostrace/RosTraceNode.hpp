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
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#pragma once

using NameAndType_t = std::map<std::string, std::vector<std::string>>;
using NodeToTopicsMap_t = std::map<std::string, NameAndType_t>;
using TopicToNodes_t = std::map<std::string, std::vector<std::string>>;

class RosTraceNode : public rclcpp::Node
{
public:
    RosTraceNode();
    std::optional<NameAndType_t> getTopicNamesAndTypes();
    std::optional<std::vector<std::string>> getNodeNames();

private:
    std::optional<NameAndType_t> topics_and_types_;
    std::optional<std::vector<std::string>> node_names_;

    std::optional<NodeToTopicsMap_t> publications_per_node_;
    std::optional<NodeToTopicsMap_t> subscriptions_per_node_;

    std::optional<TopicToNodes_t> publishers_by_topic_;
    std::optional<TopicToNodes_t> subscribers_by_topic_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> namesFromInfos(
        const std::vector<rclcpp::TopicEndpointInfo> & info) const;
    std::string cleanNodeName(const std::string & node_name) const;
};
