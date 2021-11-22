#include <BehaviourCreator.h>
#include <ConditionCreator.h>
#include <ConstraintCreator.h>
#include <PlanCreator.h>
#include <PlanAttachmentCreator.h>
#include <UtilityFunctionCreator.h>
#include <engine/AlicaContext.h>

#include <constraintsolver/CGSolver.h>
#include <clock/AlicaROSClock.h>
#include <communication/AlicaRosCommunication.h>
#include <clock/AlicaRosTimer.h>
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
    ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>(4);
    ac->addSolver<alica::reasoner::CGSolver>();
}

void Base::start()
{
    alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
            std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(),
            std::make_unique<alica::PlanCreator>(), std::make_unique<alica::PlanAttachmentCreator>());
    ac->init(creators);
}

Base::~Base()
{
    ac->terminate();
    delete ac;
    ALICATurtleWorldModel::del();
}

} // namespace turtlesim
