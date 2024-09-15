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
#include "rclcpp/rclcpp.hpp"

#include <fmt/core.h>

#include <vector>
#include <map>
#include <string>
#include <optional>

using namespace std::chrono_literals;
using NameAndType_t = std::map<std::string, std::vector<std::string>>;
using NodeToTopicsMap_t = std::map<std::string, NameAndType_t>;
using TopicToNodes_t = std::map<std::string, std::vector<std::string>>;

class J7sRostrace: public rclcpp::Node
{
public:
    J7sRostrace();
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


    std::vector<std::string> namesFromInfos(const std::vector<rclcpp::TopicEndpointInfo>& info) const;
    std::string cleanNodeName(const std::string& node_name) const;
};

J7sRostrace::J7sRostrace(): Node("rostrace"),
                            topics_and_types_{std::nullopt},
                            publications_per_node_{std::nullopt},
                            subscriptions_per_node_{std::nullopt},
                            publishers_by_topic_{std::nullopt},
                            subscribers_by_topic_{std::nullopt}
{
    // Immediately triggering timer.
    timer_ = create_timer(0ms, [this]()
    {
        RCLCPP_INFO_STREAM(get_logger(), "In timer");
        const auto graph_interface = get_node_graph_interface();

        // Get node names and topics.
        node_names_ = graph_interface->get_node_names();
        topics_and_types_ = graph_interface->get_topic_names_and_types();

        // For every node, get all of it subscriptions and publications.
        NodeToTopicsMap_t subscriptions_per_node;
        NodeToTopicsMap_t publications_per_node;
        const std::string nodeNamespace{""};
        for(const auto& node_name : node_names_.value())
        {
            RCLCPP_INFO_STREAM(get_logger(), "Node name: " << node_name);
            const auto node_name_clean = cleanNodeName(node_name); // remove leading / if present.
            const auto subscriptions = graph_interface->get_subscriber_names_and_types_by_node(node_name_clean, nodeNamespace);
            const auto publications = graph_interface->get_publisher_names_and_types_by_node(node_name_clean, nodeNamespace);

            subscriptions_per_node.emplace(node_name, subscriptions);
            publications_per_node.emplace(node_name, publications);
        }
        // Copy to member variables.
        publications_per_node_ = publications_per_node;
        subscriptions_per_node_ = subscriptions_per_node;

        TopicToNodes_t publishers_by_topic;
        TopicToNodes_t subscribers_by_topic;
        // For every topic get its publishers and subscribers.
        for(const auto& [topic, type] : topics_and_types_.value())
        {
            const auto publisher_info = graph_interface->get_publishers_info_by_topic(topic);
            const auto subscriber_info = graph_interface->get_subscriptions_info_by_topic(topic);

            publishers_by_topic.emplace(topic, namesFromInfos(publisher_info));
            subscribers_by_topic.emplace(topic, namesFromInfos(subscriber_info));
        }
        // Copy to member variables.
        publishers_by_topic_ = publishers_by_topic;
        subscribers_by_topic_ = subscribers_by_topic;

    });
}

std::optional<NameAndType_t> J7sRostrace::getTopicNamesAndTypes()
{
    return topics_and_types_;
}

std::optional<std::vector<std::string>> J7sRostrace::getNodeNames()
{
    return node_names_;
}

std::vector<std::string> J7sRostrace::namesFromInfos(const std::vector<rclcpp::TopicEndpointInfo>& infos) const
{
    std::vector<std::string> names;
    for(const auto& info : infos)
    {
        names.emplace_back(info.node_name());
    }
    return names;
}

std::string J7sRosRostrace::cleanNodeName(const std::string& node_name) const {
    // Strip leading / if present.
    std::string node_name_clean = node_name;
    if(node_name_clean.at(0) == '/') {
        node_name_clean.erase(0, 1);
    }
    return node_name_clean;
}



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    auto node = std::make_shared<J7sRostrace>();
    exec.add_node(node);
    // Spin once.
    exec.spin_some();

    const auto topics_and_types = node->getTopicNamesAndTypes();
    const auto node_names = node->getNodeNames();
    if(not topics_and_types or not node_names)
    {
        fmt::print("I never got anything.\n");
        return -1;
    }
    for(auto [topic, types] : topics_and_types.value())
    {
        fmt::print("topic: {} -> type: {}\n", topic, types[0]);
    }
    for(auto node : node_names.value())
    {
        fmt::print("Node: {}\n", node);
    }
    rclcpp::shutdown();
    return 0;
}
