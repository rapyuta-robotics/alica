#include "Creators.h"
#include "Profiling.h"

#include <communication/AlicaDummyCommunication.h>
#include <engine/AlicaContext.h>
#include <engine/AlicaTimer.h>
#include <engine/blackboard/Blackboard.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{

/**
 * Builds one fully-wired ALICA agent. Each agent is an independent
 * AlicaContext; they find each other through the communicator.
 */
std::unique_ptr<alica::AlicaContext> makeAgent(const std::string& configPath, const std::string& agentName, alica::AgentId agentId)
{
    ZoneScoped;

    // Note the explicit vector: a braced `{configPath}` would select the
    // deprecated single-path overload instead.
    const std::vector<std::string> configPaths{configPath};
    alica::AlicaContextParams params(agentName, // local agent name
            configPaths,                        // folders scanned for Alica.yaml + model files
            "Roleset",                          // roleset to use
            "MinimalMaster",                    // master plan
            false,                              // stepEngine: false -> engine runs its own loop
            agentId);

    std::unique_ptr<alica::AlicaContext> ac;
    {
        // Parsing and validating the .pml/.beh/.rst model files happens here.
        ZoneScopedN("AlicaContext ctor (model loading)");
        ac = std::make_unique<alica::AlicaContext>(params);
    }
    if (!ac->isValid()) {
        return nullptr;
    }

    // Platform pieces. Swap these for the ROS 1 / ROS 2 proxies to get
    // multicast over ROS topics and a ROS clock instead.
    ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
    ac->setTimerFactory<alica::AlicaSystemTimerFactory>();       // drives behaviours/plans
    ac->setEngineTimerFactory<alica::AlicaSystemTimerFactory>(); // drives the engine loop
    // No setLogger call: the AlicaContext constructor installs an AlicaDefaultLogger if the
    // process has none yet, and the context that installed it re-configures it from the config.
    // The logger is process-wide, so with several contexts every agent's logs carry the first
    // agent's name. Run one agent per process if you want per-agent log attribution.

    // Seed the global blackboard before init so the first tick sees the key.
    alica::LockedBlackboardRW(*ac->getGlobalBlackboardShared()).set<int64_t>(minimal::CountUp::COUNTER_KEY, 0);

    alica::AlicaCreators creators(std::make_unique<minimal::ConditionCreator>(), std::make_unique<minimal::UtilityFunctionCreator>(),
            std::make_unique<minimal::ConstraintCreator>(), std::make_unique<minimal::BehaviourCreator>(), std::make_unique<minimal::PlanCreator>(),
            std::make_unique<minimal::TransitionConditionCreator>());

    {
        ZoneScopedN("AlicaContext::init");
        if (ac->init(std::move(creators)) != 0) {
            return nullptr;
        }
    }
    return ac;
}

} // namespace

int main(int argc, char** argv)
{
    tracy::SetThreadName("main");

    const std::string configPath = (argc > 1) ? argv[1] : "etc";
    const int agentCount = (argc > 2) ? std::stoi(argv[2]) : 2;

    const std::string appInfo = "minimal_alica, " + std::to_string(agentCount) + " agent(s), config " + configPath;
    TracyAppInfo(appInfo.c_str(), appInfo.size());

    // The master plan's entrypoint allows 1-2 agents on DefaultTask, so a team
    // of two forms a complete assignment.
    std::vector<std::unique_ptr<alica::AlicaContext>> agents;
    for (int i = 0; i < agentCount; ++i) {
        const std::string name = "agent" + std::string(1, static_cast<char>('A' + i));
        auto ac = makeAgent(configPath, name, 1000 + i);
        if (!ac) {
            std::cerr << "Failed to start " << name << std::endl;
            return 1;
        }
        std::cout << "started " << ac->getLocalAgentName() << " (id " << ac->getLocalAgentId() << ")" << std::endl;
        agents.push_back(std::move(ac));
    }

    // The engines tick on their own threads; just stay alive for a while.
    {
        ZoneScopedN("agents running");
        std::this_thread::sleep_for(std::chrono::seconds(6));
    }

    for (const auto& ac : agents) {
        const int64_t count = alica::LockedBlackboardRO(*ac->getGlobalBlackboardShared()).get<int64_t>(minimal::CountUp::COUNTER_KEY);
        std::cout << ac->getLocalAgentName() << " final counter = " << count << std::endl;
    }

    for (const auto& ac : agents) {
        ac->terminate();
    }
    return 0;
}
