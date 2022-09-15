//#include <alica/BehaviourCreator.h>
#include <alica/ConditionCreator.h>
#include <alica/ConstraintCreator.h>
#include <alica/DynamicBehaviourCreator.h>
#include <alica/DynamicPlanCreator.h>
#include <alica/TransitionConditionCreator.h>
#include <alica/UtilityFunctionCreator.h>
#include <engine/AlicaContext.h>

#include <clock/AlicaROSClock.h>
#include <clock/AlicaRosTimer.h>
#include <communication/AlicaRosCommunication.h>
#include <constraintsolver/CGSolver.h>
#include <ros/ros.h>

#include <boost/dll/import.hpp> // for import_alias

#include <alica_ros_turtlesim/base.hpp>

namespace turtlesim
{

Base::Base(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, const std::string& name, const int agent_id, const std::string& roleset,
        const std::string& master_plan, const std::string& path)
        : spinner(0)
{
    _libraryPath = path + "/../../../../../devel/lib/libalica_customer_library.so";

    // create world model
    ALICATurtleWorldModelCallInit(nh, priv_nh);
    // Initialize Alica
    ac = new alica::AlicaContext(AlicaContextParams(name, path + "/etc/", roleset, master_plan, false, agent_id));

    ac->setCommunicator<alicaRosProxy::AlicaRosCommunication>();
    ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>();
}

void Base::ALICATurtleWorldModelCallInit(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
{
    if (!boost::filesystem::exists(_libraryPath)) {
        std::cerr << "Error:"
                  << "Lib not exixts in this path:" << _libraryPath << std::endl;
        return;
    } else {
        std::cerr << "Debug:"
                  << "Lib exixts in this path:" << _libraryPath << " for ALICATurtleWorldModelInit" << std::endl;
    }

    typedef void(InitType)(ros::NodeHandle&, ros::NodeHandle&);
    std::function<InitType> wminit;
    wminit = boost::dll::import_alias<InitType>(      // type of imported symbol must be explicitly specified
            _libraryPath,                             // complete path to library also with file name
            "ALICATurtleWorldModelInit",              // symbol to import
            boost::dll::load_mode::append_decorations // do append extensions and prefixes
    );
    wminit(nh, priv_nh);
}

void Base::start()
{
    alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
            std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(), std::make_unique<alica::DynamicPlanCreator>(),
            std::make_unique<alica::TransitionConditionCreator>());

    spinner.start(); // start spinner before initializing engine, but after setting context
    ac->init(std::move(creators));
    ac->addSolver<alica::reasoner::CGSolver>();
}

Base::~Base()
{
    spinner.stop(); // stop spinner before terminating engine
    ac->terminate();
    delete ac;
    // ALICATurtleWorldModel::del(); luca
}

} // namespace turtlesim
