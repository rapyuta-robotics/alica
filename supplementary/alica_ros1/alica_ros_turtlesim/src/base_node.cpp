#include <chrono>
#include <ros/ros.h>
#include <signal.h>
#include <thread>

#include <alica_ros_turtlesim/base.hpp>
#include <turtlesim/Kill.h>
#include <turtlesim/Spawn.h>

void killMyTurtle(const std::string& name, ros::NodeHandle& privNh)
{
    ros::ServiceClient killClient = privNh.serviceClient<turtlesim::Kill>("/kill");
    turtlesim::Kill killSrv;
    killSrv.request.name = name;
    if (killClient.waitForExistence() && killClient.call(killSrv)) {
        ROS_INFO_STREAM(name << " was killed");
    } else {
        ROS_INFO_STREAM(name << " does not exist or failed to kill " << name);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "alica_turtle_base_node");
    ROS_INFO("Started Turtle Base Node.");
    std::string name, roleset, masterPlan, alicaPath;
    int agentId;

    ros::NodeHandle nh, privNh("~");
    privNh.getParam("name", name);
    privNh.getParam("roleset", roleset);
    privNh.getParam("master_plan", masterPlan);
    privNh.getParam("alica_path", alicaPath);
    privNh.getParam("agent_id", agentId);

    ROS_INFO_STREAM("HostName    : " << name);
    ROS_INFO_STREAM("Roleset     : " << (roleset.empty() ? "Default" : roleset));
    ROS_INFO_STREAM("Master Plan : " << masterPlan);
    ROS_INFO_STREAM("ALICA Path  : " << alicaPath);
    ROS_INFO_STREAM("Agent ID    : " << agentId);

    if (masterPlan.size() == 0) {
        ROS_ERROR_STREAM("Master plan or roleset location is not available");
        return 0;
    }

    // kill turtle if same name turtle already exist
    killMyTurtle(name, privNh);

    turtlesim::Base base(nh, privNh, name, agentId, roleset, masterPlan, alicaPath);

    ROS_INFO("Starting ALICA turtle Base.......");
    base.start();

    int counter = 3000;

    while (counter-- > 0) {
        ros::Rate(10).sleep();
    }

    // Ensure turtle is no longer visible
    killMyTurtle(name, privNh);
    return 0;
}
