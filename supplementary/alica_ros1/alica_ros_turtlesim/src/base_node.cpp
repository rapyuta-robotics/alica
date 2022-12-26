#include <chrono>
#include <ros/ros.h>
#include <signal.h>
#include <thread>

#include <alica_ros_turtlesim/base.hpp>
#include <turtlesim/Kill.h>
#include <turtlesim/Spawn.h>

void killMyTurtle(const std::string& name, ros::NodeHandle& priv_nh)
{
    ros::ServiceClient kill_client = priv_nh.serviceClient<turtlesim::Kill>("/kill");
    turtlesim::Kill kill_srv;
    kill_srv.request.name = name;
    if (kill_client.waitForExistence() && kill_client.call(kill_srv)) {
        ROS_INFO_STREAM(name << " was killed");
    } else {
        ROS_INFO_STREAM(name << " does not exist or failed to kill " << name);
    }
}

bool spawnMyTurtle(const std::string& name, ros::NodeHandle& priv_nh)
{
    ros::ServiceClient spawn_client = priv_nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 1;
    spawn_srv.request.y = 1;
    spawn_srv.request.theta = 0;
    spawn_srv.request.name = name;
    if (spawn_client.waitForExistence() && spawn_client.call(spawn_srv)) {
        ROS_INFO_STREAM(name << " was spawn");
    } else {
        ROS_ERROR_STREAM("Failed to spawn " << name);
        return false;
    }

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "alica_turtle_base_node");
    ROS_INFO("Started Turtle Base Node.");
    std::string name, roleset, master_plan, alica_path;
    int agent_id;
    bool dynamic;

    ros::NodeHandle nh, priv_nh("~");
    priv_nh.getParam("name", name);
    priv_nh.getParam("roleset", roleset);
    priv_nh.getParam("master_plan", master_plan);
    priv_nh.getParam("alica_path", alica_path);
    priv_nh.getParam("agent_id", agent_id);
    priv_nh.getParam("dynamic", dynamic);

    ROS_INFO_STREAM("HostName    : " << name);
    ROS_INFO_STREAM("Roleset     : " << (roleset.empty() ? "Default" : roleset));
    ROS_INFO_STREAM("Master Plan : " << master_plan);
    ROS_INFO_STREAM("ALICA Path  : " << alica_path);
    ROS_INFO_STREAM("Agent ID    : " << agent_id);
    ROS_INFO_STREAM("Dynamic     : " << dynamic);

    if (master_plan.size() == 0) {
        ROS_ERROR_STREAM("Master plan or roleset location is not available");
        return 0;
    }

    // kill turtle if same name turtle already exist
    killMyTurtle(name, priv_nh);

    // spawn turtle in sim
    if (!spawnMyTurtle(name, priv_nh)) {
        return 1;
    }

    if (dynamic) {
        ROS_INFO("Creating ALICA turtle Base in dynamic loading mode.......");
    } else {
        ROS_INFO("Creating ALICA turtle Base.......");
    }
    turtlesim::Base base(name, agent_id, roleset, master_plan, alica_path, dynamic);

    ROS_INFO("Starting ALICA turtle Base.......");
    base.start(nh, priv_nh);

    // Wait for ctrl+c
    sigset_t signal_set;
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGINT);
    int sig_number;
    if (sigwait(&signal_set, &sig_number) != 0) {
        ROS_ERROR_STREAM("<main> Error sigwait()");
        exit(1);
    }

    // Ensure turtle is no longer visible
    killMyTurtle(name, priv_nh);
    return 0;
}
