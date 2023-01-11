#include <DynamicBehaviourCreator.h>
#include <DynamicConditionCreator.h>
#include <DynamicLoadingUtils.h>
#include <DynamicPlanCreator.h>
#include <DynamicTransitionConditionCreator.h>
#include <alica/BehaviourCreator.h>
#include <alica/ConditionCreator.h>
#include <alica/ConstraintCreator.h>
#include <alica/PlanCreator.h>
#include <alica/TransitionConditionCreator.h>
#include <alica/UtilityFunctionCreator.h>
#include <alica_ros_turtlesim/base.hpp>
#include <alica_ros_turtlesim/world_model.hpp>
#include <boost/dll/import.hpp> // for import_alias
#include <clock/AlicaROSClock.h>
#include <clock/AlicaRosTimer.h>
#include <communication/AlicaRosCommunication.h>
#include <constraintsolver/CGSolver.h>
#include <engine/AlicaContext.h>
#include <engine/logging/AlicaDefaultLogger.h>
#include <logger/AlicaRosLogger.h>
#include <ros/ros.h>

namespace turtlesim
{

Base::Base(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, const std::string& name, const int agent_id, const std::string& roleset,
        const std::string& master_plan, const std::string& path, bool loadDynamically)
        : spinner(0)
        , _loadDynamically(loadDynamically)
{
    // Initialize Alica
    ac = new alica::AlicaContext(AlicaContextParams(name, path + "/etc/", roleset, master_plan, false, agent_id));

    ac->setCommunicator<alicaRosProxy::AlicaRosCommunication>();
    ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>();
    ac->setLogger<alicaRosLogger::AlicaRosLogger>(agent_id);
    // create world model
    if (_loadDynamically) {
        ALICASetWorldModel(nh, priv_nh);
    } else {
        alica::LockedBlackboardRW(ac->editGlobalBlackboard()).set("worldmodel", std::make_shared<turtlesim::ALICATurtleWorldModel>(nh, priv_nh));
    }
}

void Base::ALICASetWorldModel(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
{
    std::vector<std::string> tmp = calculateLibraryPath();
    std::string libraryPath = calculateLibraryCompleteName(tmp, "alica_turtlesim_library");
    if (libraryPath.empty()) {
        std::cerr << "Error:"
                  << "Lib not exists" << std::endl;
        return;
    }

    typedef void(InitType)(alica::AlicaContext*, ros::NodeHandle&, ros::NodeHandle&);
    std::function<InitType> setWm;
    setWm = boost::dll::import_alias<InitType>(       // type of imported symbol must be explicitly specified
            libraryPath,                              // complete path to library also with file name
            "setWorldModel",                          // symbol to import
            boost::dll::load_mode::append_decorations // do append extensions and prefixes
    );
    setWm(ac, nh, priv_nh);
}

void Base::start()
{
    if (_loadDynamically) {
        alica::AlicaCreators creators(std::make_unique<DynamicConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(), std::make_unique<alica::DynamicPlanCreator>(),
                std::make_unique<alica::DynamicTransitionConditionCreator>());

        spinner.start(); // start spinner before initializing engine, but after setting context
        ac->init(std::move(creators), false);
        ac->addSolver<alica::reasoner::CGSolver>();
    } else {
        alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
                std::make_unique<alica::TransitionConditionCreator>());
        spinner.start();                      // start spinner before initializing engine, but after setting context
        ac->init(std::move(creators), false); // Do not start engine, I need to add WM before
        ac->addSolver<alica::reasoner::CGSolver>();
    }
}

Base::~Base()
{
    spinner.stop(); // stop spinner before terminating engine
    ac->terminate();
    delete ac;
}

} // namespace turtlesim
