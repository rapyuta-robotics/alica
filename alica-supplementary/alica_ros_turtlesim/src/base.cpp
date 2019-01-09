#include <CGSolver.h>
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

    // ALICA essentials setting
    essentials::SystemConfig& sc = essentials::SystemConfig::getInstance();
    sc.setHostname(name);
    sc.setRootPath(path);
    sc.setConfigPath(path + "/etc");

    // ALICA engine setting
    ae = new alica::AlicaEngine(new essentials::AgentIDManager(new essentials::AgentIDFactory()), roleset, master_plan, false);
    bc = new alica::BehaviourCreator();
    cc = new alica::ConditionCreator();
    uc = new alica::UtilityFunctionCreator();
    crc = new alica::ConstraintCreator();
    ae->setAlicaClock(new alica::AlicaClock());
    ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
    ae->addSolver(new alica::reasoner::CGSolver(ae));
    ae->init(bc, cc, uc, crc);
}

void Base::start()
{
    ae->start();
}

Base::~Base()
{
    ae->shutdown();
    delete ae->getCommunicator();
    delete ae;
    delete cc;
    delete bc;
    delete uc;
    delete crc;
    ALICATurtleWorldModel::del();
}

} // namespace turtlesim