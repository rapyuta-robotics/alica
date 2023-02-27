#include <chrono>
#include <signal.h>
#include <thread>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <alica_ros2_turtlesim/base.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::string name, roleset, master_plan, alica_path;
    int agent_id;

    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("alica_ros2_turtlesim");

    nh->declare_parameter("name", "Hello!");
    nh->declare_parameter("roleset", "Default");
    nh->declare_parameter("master_plan", "Master");
    nh->declare_parameter("alica_path", "/");
    nh->declare_parameter("agent_id", 0);

    nh->get_parameter("name", name);
    nh->get_parameter("roleset", roleset);
    nh->get_parameter("master_plan", master_plan);
    nh->get_parameter("alica_path", alica_path);
    nh->get_parameter("agent_id", agent_id);

    RCLCPP_INFO(nh->get_logger(), "HostName    : %s", name.c_str());
    RCLCPP_INFO(nh->get_logger(), "Roleset     : %s", (roleset.empty() ? "Default" : roleset.c_str()));
    RCLCPP_INFO(nh->get_logger(), "Master Plan : %s", master_plan.c_str());
    RCLCPP_INFO(nh->get_logger(), "ALICA Path  : %s", alica_path.c_str());
    RCLCPP_INFO(nh->get_logger(), "Agent ID    : %d", agent_id);

    if (master_plan.size() == 0) {
        RCLCPP_ERROR(nh->get_logger(), "Master plan or roleset location is not available");
        return 0;
    }

    RCLCPP_INFO(nh->get_logger(), "Creating ALICA turtle Base.......");
    turtlesim::Base base(nh, name, agent_id, roleset, master_plan, alica_path);

    RCLCPP_INFO(nh->get_logger(), "Starting ALICA turtle Base.......");
    base.start();

    // Wait for ctrl+c
    sigset_t signal_set;
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGINT);
    int sig_number;
    if (sigwait(&signal_set, &sig_number) != 0) {
        RCLCPP_ERROR(nh->get_logger(), "<main> Error sigwait()");
        exit(1);
    }

    // Ensure turtle is no longer visible
    // killMyTurtle(name, priv_nh);
    return 0;
}
