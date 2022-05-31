#include <BehaviourCreator.h>
#include <ConditionCreator.h>
#include <ConstraintCreator.h>
#include <PlanCreator.h>
#include <UtilityFunctionCreator.h>
#include <engine/AlicaContext.h>

#include <clock/AlicaROSClock.h>
#include <clock/AlicaRosTimer.h>
#include <communication/AlicaRosCommunication.h>
#include <constraintsolver/CGSolver.h>
#include <ros/ros.h>

#include <alica_ros_turtlesim/base.hpp>

namespace turtlesim
{

Base::Base(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, const std::string& name, const int agent_id, const std::string& roleset,
        const std::string& master_plan, const std::string& path)
        : spinner(0)
{
    // create world model
    ALICATurtleWorldModel::init(nh, priv_nh);
    // Initialize Alica
    ac = new alica::AlicaContext(AlicaContextParams(name, path + "/etc/", roleset, master_plan, false, agent_id));

    ac->setCommunicator<alicaRosProxy::AlicaRosCommunication>();
    ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>();
    ac->addSolver<alica::reasoner::CGSolver>();
}

void Base::start()
{
    alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
            std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>());

    spinner.start(); // start spinner before initializing engine, but after setting context
    ac->init(std::move(creators));
}

Base::~Base()
{
    spinner.stop(); // stop spinner before terminating engine
    ac->terminate();
    delete ac;
    ALICATurtleWorldModel::del();
}

} // namespace turtlesim
