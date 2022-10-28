#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <thread>

#include <alica_ros2_turtlesim/base.hpp>
#include <alica_ros2_turtlesim/world_model.hpp>

#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // ros::init(argc, argv, "alica_turtle_base_node");
    // ROS_INFO("Started Turtle Base Node.");
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

    rclcpp::Node::SharedPtr priv_nh = rclcpp::Node::make_shared("ros2_turtlesim");
    priv_nh->declare_parameter("name", "Hello!");

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "HostName    : %s", name.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Roleset     : %s", (roleset.empty() ? "Default" : roleset.c_str()));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Master Plan : %s", master_plan.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ALICA Path  : %s", alica_path.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Agent ID    : %d", agent_id);

    if (master_plan.size() == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Master plan or roleset location is not available");
        return 0;
    }
    // kill turtle if same name turtle already exist
    // killMyTurtle(name, priv_nh);

    // // spawn turtle in sim
    // if (!spawnMyTurtle(name, priv_nh))
    //     return 1;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating ALICA turtle Base.......");
    turtlesim::Base base(nh, priv_nh, name, agent_id, roleset, master_plan, alica_path);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting ALICA turtle Base.......");
    base.start();

    // Wait for ctrl+c
    sigset_t signal_set;
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGINT);
    int sig_number;
    if (sigwait(&signal_set, &sig_number) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "<main> Error sigwait()");
        exit(1);
    }

    // Ensure turtle is no longer visible
    // killMyTurtle(name, priv_nh);
    return 0;
}
