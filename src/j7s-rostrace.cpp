// Copyright 2023 James Pace
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

class J7sRostrace: public rclcpp::Node
{
public:
    J7sRostrace();
    std::optional<NameAndType_t> getTopicNamesAndTypes();
private:
    std::optional<NameAndType_t> topics_and_types_;

    rclcpp::TimerBase::SharedPtr timer_;

};

J7sRostrace::J7sRostrace(): Node("rostrace"),
                            topics_and_types_{std::nullopt}
{
    timer_ = create_timer(0ms, [this]()
    {
        RCLCPP_WARN_STREAM(get_logger(), "In timer");
        if(not topics_and_types_)
        {
            topics_and_types_ = get_topic_names_and_types();
        }
    });
}

std::optional<NameAndType_t> J7sRostrace::getTopicNamesAndTypes()
{
    return topics_and_types_;
}



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    auto node = std::make_shared<J7sRostrace>();
    exec.add_node(node);
    for(int cnt = 0; cnt < 10; cnt++)
    {
        exec.spin_some();
    }

    const auto topics_and_types = node->getTopicNamesAndTypes();
    if(not topics_and_types)
    {
        fmt::print("I never got anything.\n");
        return -1;
    }
    for(auto [topic, types] : topics_and_types.value())
    {
        fmt::print("topic: {} -> type: {}\n", topic, types[0]);
    }
    rclcpp::shutdown();
    return 0;
}
