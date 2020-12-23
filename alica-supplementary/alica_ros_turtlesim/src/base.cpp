#include <BehaviourCreator.h>
#include <ConditionCreator.h>
#include <ConstraintCreator.h>
#include <UtilityFunctionCreator.h>
#include <engine/AlicaContext.h>

#include <constraintsolver/CGSolver.h>
#include <clock/AlicaROSClock.h>
#include <communication/AlicaRosCommunication.h>
#include <ros/ros.h>

#include <alica_ros_turtlesim/base.hpp>

namespace turtlesim
{

Base::Base(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, const std::string& name, const std::string& roleset, const std::string& master_plan,
        const std::string& path)
{
    // create world model
    ALICATurtleWorldModel::init(nh, priv_nh);
    // Initialize Alica
    ac = new alica::AlicaContext(AlicaContextParams(name, path + "/etc/", roleset, master_plan, false));
    ac->setCommunicator<alicaRosProxy::AlicaRosCommunication>();
    ac->addSolver<alica::reasoner::CGSolver>();
}

void Base::start()
{
    alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
            std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>());
    ac->init(creators);
}

Base::~Base()
{
    ac->terminate();
    delete ac;
    ALICATurtleWorldModel::del();
}

} // namespace turtlesim
