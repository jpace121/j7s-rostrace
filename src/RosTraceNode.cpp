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
#include <j7s-rostrace/RosTraceNode.hpp>

using namespace std::chrono_literals;

RosTraceNode::RosTraceNode() :
    Node("rostrace"),
    topics_and_types_{std::nullopt},
    publications_per_node_{std::nullopt},
    subscriptions_per_node_{std::nullopt},
    publishers_by_topic_{std::nullopt},
    subscribers_by_topic_{std::nullopt}
{
    // Immediately triggering timer.
    timer_ = create_timer(
        0ms,
        [this]()
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
            for (const auto & node_name : node_names_.value())
            {
                RCLCPP_INFO_STREAM(get_logger(), "Node name: " << node_name);
                const auto node_name_clean =
                    cleanNodeName(node_name);  // remove leading / if present.
                const auto subscriptions = graph_interface->get_subscriber_names_and_types_by_node(
                    node_name_clean, nodeNamespace);
                const auto publications = graph_interface->get_publisher_names_and_types_by_node(
                    node_name_clean, nodeNamespace);

                subscriptions_per_node.emplace(node_name, subscriptions);
                publications_per_node.emplace(node_name, publications);
            }
            // Copy to member variables.
            publications_per_node_ = publications_per_node;
            subscriptions_per_node_ = subscriptions_per_node;

            TopicToNodes_t publishers_by_topic;
            TopicToNodes_t subscribers_by_topic;
            // For every topic get its publishers and subscribers.
            for (const auto & [topic, type] : topics_and_types_.value())
            {
                const auto publisher_info = graph_interface->get_publishers_info_by_topic(topic);
                const auto subscriber_info =
                    graph_interface->get_subscriptions_info_by_topic(topic);

                publishers_by_topic.emplace(topic, namesFromInfos(publisher_info));
                subscribers_by_topic.emplace(topic, namesFromInfos(subscriber_info));
            }
            // Copy to member variables.
            publishers_by_topic_ = publishers_by_topic;
            subscribers_by_topic_ = subscribers_by_topic;
        });
}

std::optional<NameAndType_t> RosTraceNode::getTopicNamesAndTypes()
{
    return topics_and_types_;
}

std::optional<std::vector<std::string>> RosTraceNode::getNodeNames()
{
    return node_names_;
}

std::vector<std::string> RosTraceNode::namesFromInfos(
    const std::vector<rclcpp::TopicEndpointInfo> & infos) const
{
    std::vector<std::string> names;
    for (const auto & info : infos)
    {
        names.emplace_back(info.node_name());
    }
    return names;
}

std::string RosTraceNode::cleanNodeName(const std::string & node_name) const
{
    // Strip leading / if present.
    std::string node_name_clean = node_name;
    if (node_name_clean.at(0) == '/')
    {
        node_name_clean.erase(0, 1);
    }
    return node_name_clean;
}
