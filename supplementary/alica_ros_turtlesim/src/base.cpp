//#include <alica/BehaviourCreator.h>
#include <alica/ConditionCreator.h>
#include <alica/ConstraintCreator.h>
#include <alica/DynamicBehaviourCreator.h>
#include <alica/PlanCreator.h>
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
    // create world model
    //ALICATurtleWorldModel::init(nh, priv_nh);
    ALICATurtleWorldModelCallInit(nh, priv_nh);
    // Initialize Alica
    ac = new alica::AlicaContext(AlicaContextParams(name, path + "/etc/", roleset, master_plan, false, agent_id));

    ac->setCommunicator<alicaRosProxy::AlicaRosCommunication>();
    ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>();
}

void Base::ALICATurtleWorldModelCallInit(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
{
    std::string libraryPath = "/var/tmp/customers/libalica_customer_library.so";

    /*
    if (context.behaviourModel->getLibraryName() == "") {
        std::cerr << "Error:"
                  << "Empty library name for" << context.behaviourModel->getName() << std::endl;
        return nullptr;
    }

    std::string libraryPath = context.libraryPath + "/lib" + context.behaviourModel->getLibraryName() + ".so";
    if (!boost::filesystem::exists(libraryPath)) {
        std::cerr << "Error:"
                  << "Lib not exixts in this path:" << libraryPath << std::endl;
        return nullptr;
    } else {
        std::cerr << "Debug:"
                  << "Lib exixts in this path:" << libraryPath << " for:" << context.behaviourModel->getName() << " for:" << behaviourId << std::endl;
    }
    */
    typedef void(InitType)(ros::NodeHandle&, ros::NodeHandle&);
    std::function<InitType> wminit;
    wminit = boost::dll::import_alias<InitType>(        // type of imported symbol must be explicitly specified
            libraryPath,                              // complete path to library also with file name
            "ALICATurtleWorldModelInit",              // symbol to import
            boost::dll::load_mode::append_decorations // do append extensions and prefixes
    );
    wminit(nh,priv_nh);
}

void Base::start()
{
    /*alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
            std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
            std::make_unique<alica::TransitionConditionCreator>());
            */
    alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
            std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
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
    //ALICATurtleWorldModel::del(); luca
}

} // namespace turtlesim
