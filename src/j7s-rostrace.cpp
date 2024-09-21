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

#include <functional>
#include <iostream>

#include <j7s-rostrace/RosTraceNode.hpp>

#include "rclcpp/rclcpp.hpp"

class UI
{
public:
    UI(const std::vector<std::string>& node_names);
    void showMainPage();
    void showNodeListPage();
    void showErrorPage(std::function<void(void)> calling_function);
private:
    std::tuple<char, std::string> getInput();

    const std::vector<std::string> node_names_;
};

UI::UI(const std::vector<std::string>& node_names):
    node_names_{node_names}
{
}

std::tuple<char, std::string> UI::getInput()
{
    std::cout << "Selection: ";
    std::string input;
    std::getline(std::cin, input);
    std::cout << "\n";
    if(input.length() <= 2)
    {
        return std::make_tuple(input[0], std::string());
    }
    return std::make_tuple(input[0], input.substr(2, std::string::npos));
}

void UI::showMainPage()
{
    constexpr char QUIT_OPTION = 'q';
    constexpr char SEE_ALL_NODES_OPTION ='0';
    constexpr char SEE_ALL_TOPICS_OPTION = '1';
    fmt::print("Welcome! \n\n");
    fmt::print("{} \t See all nodes. \n", SEE_ALL_NODES_OPTION);
    fmt::print("{} \t See all topics. \n", SEE_ALL_TOPICS_OPTION);
    fmt::print("{} \t Quit. \n", QUIT_OPTION);
    fmt::print("\n");

    const auto [selection, options] = getInput();
    switch(selection)
    {
        case SEE_ALL_NODES_OPTION:
            showNodeListPage();
            break;
        case SEE_ALL_TOPICS_OPTION:
            break;
        case QUIT_OPTION:
            fmt::print("Quit");
            exit(0);
            break;
        default:
            showErrorPage([this](){showMainPage();});
            break;
    }
}

void UI::showErrorPage(std::function<void(void)> calling_function)
{
    fmt::print("Unrecognized option.\n Try something else.\n");
    fmt::print("\n\n");
    calling_function();
}

void UI::showNodeListPage()
{
    fmt::print("All nodes. \n");
    for(size_t index = 0; index < node_names_.size(); index++)
    {
        fmt::print("{} \t {}\n", index, node_names_[index]);
    }
    fmt::print("\n\n");
    fmt::print("Option: ");
}

UI run_ros(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    auto node = std::make_shared<RosTraceNode>();
    exec.add_node(node);
    // Spin once.
    exec.spin_some();
    rclcpp::shutdown();

    // Build UI.
    UI ui(*node->getNodeNames()); //TODO: Handle optional better.

    return ui;
}

int main(int argc, char ** argv)
{
    auto ui = run_ros(argc, argv);

    ui.showMainPage();


    /*
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
    */
    return 0;
}
