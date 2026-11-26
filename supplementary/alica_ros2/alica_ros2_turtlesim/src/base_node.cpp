#include <cerrno>
#include <chrono>
#include <pthread.h>
#include <signal.h>
#include <thread>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <alica_ros2_turtlesim/base.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    // Block SIGINT and SIGTERM before any thread starts, so that every thread inherits the mask and only the sigtimedwait() at the
    // end of main receives them: sigtimedwait() only reliably receives signals that every thread blocks.
    sigset_t signal_set;
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGINT);
    sigaddset(&signal_set, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &signal_set, nullptr);

    rclcpp::init(argc, argv);
    std::string name, roleset, master_plan, alica_path;
    int agent_id;
    bool dedicated_engine_thread = true;

    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("alica_ros2_turtlesim");

    nh->declare_parameter("name", "Hello!");
    nh->declare_parameter("roleset", "Default");
    nh->declare_parameter("master_plan", "Master");
    nh->declare_parameter("alica_path", "/");
    nh->declare_parameter("agent_id", 0);
    nh->declare_parameter("dedicated_engine_thread", true);

    nh->get_parameter("name", name);
    nh->get_parameter("roleset", roleset);
    nh->get_parameter("master_plan", master_plan);
    nh->get_parameter("alica_path", alica_path);
    nh->get_parameter("agent_id", agent_id);
    nh->get_parameter("dedicated_engine_thread", dedicated_engine_thread);

    RCLCPP_INFO(nh->get_logger(), "HostName    : %s", name.c_str());
    RCLCPP_INFO(nh->get_logger(), "Roleset     : %s", (roleset.empty() ? "Default" : roleset.c_str()));
    RCLCPP_INFO(nh->get_logger(), "Master Plan : %s", master_plan.c_str());
    RCLCPP_INFO(nh->get_logger(), "ALICA Path  : %s", alica_path.c_str());
    RCLCPP_INFO(nh->get_logger(), "Agent ID    : %d", agent_id);
    RCLCPP_INFO(nh->get_logger(), "Dedicated engine thread : %s", dedicated_engine_thread ? "true" : "false");

    if (master_plan.size() == 0) {
        RCLCPP_ERROR(nh->get_logger(), "Master plan or roleset location is not available");
        return 0;
    }

    RCLCPP_INFO(nh->get_logger(), "Creating ALICA turtle Base.......");
    turtlesim::Base base(nh, name, agent_id, roleset, master_plan, alica_path, dedicated_engine_thread);

    RCLCPP_INFO(nh->get_logger(), "Starting ALICA turtle Base.......");
    base.start();

    // Wait for ctrl+c, or SIGTERM from ros2 launch or docker stop. Without a dedicated engine thread, this loop also runs the
    // engine, at about 10 Hz.
    const timespec loop_period{0, 100000000};
    while (true) {
        if (!dedicated_engine_thread) {
            base.spinEngineOnce();
        }
        const int sig_number = sigtimedwait(&signal_set, nullptr, &loop_period);
        if (sig_number == SIGINT || sig_number == SIGTERM) {
            break;
        }
        if (sig_number < 0 && errno != EAGAIN && errno != EINTR) {
            RCLCPP_ERROR(nh->get_logger(), "<main> Error sigtimedwait()");
            exit(1);
        }
    }

    // Base::~Base() kills the turtle, so it is no longer visible
    return 0;
}
