#include <engine/ConfigChangeListener.h>
#include <engine/PlanRepository.h>
#include <engine/logging/AlicaDefaultLogger.h>
#include <engine/logging/AlicaLogger.h>
#include <engine/model/AlicaElement.h>
#include <engine/modelmanagement/ModelManager.h>

#include <yaml-cpp/yaml.h>

#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace
{
void usage()
{
    std::cerr << "usage: alica_load_plans [--placeholder-mapping FILE] MASTER_PLAN ROLE_SET DIR...\n"
                 "Parses MASTER_PLAN and everything it references, and ROLE_SET, from the plan files in DIR...\n"
                 "the same way the engine does at startup. Exits non-zero if the engine rejects them.\n";
}
} // namespace

int main(int argc, char** argv)
{
    std::vector<std::string> args(argv + 1, argv + argc);
    std::optional<std::string> placeholderMapping;
    if (args.size() >= 2 && args[0] == "--placeholder-mapping") {
        std::ifstream in(args[1]);
        if (!in) {
            std::cerr << "cannot read " << args[1] << "\n";
            return 2;
        }
        std::stringstream content;
        content << in.rdbuf();
        placeholderMapping = content.str();
        args.erase(args.begin(), args.begin() + 2);
    }
    if (args.size() < 3) {
        usage();
        return 2;
    }
    const std::string masterPlan = args[0];
    const std::string roleSet = args[1];
    const std::vector<std::string> dirs(args.begin() + 2, args.end());

    alica::AlicaLogger::set<alica::AlicaDefaultLogger>(alica::Verbosity::WARNING, "alica_load_plans");
    YAML::Node config;
    config["Alica"]["CycleDetection"]["MinimalAuthorityTimeInterval"] = 800;
    alica::ConfigChangeListener configChangeListener(config);
    alica::PlanRepository planRepository;
    alica::ModelManager modelManager(configChangeListener, dirs, planRepository);
    try {
        modelManager.loadPlanTree(masterPlan, placeholderMapping);
        modelManager.loadRoleSet(roleSet);
    } catch (const std::exception& e) {
        std::cerr << "failed to load " << masterPlan << ": " << e.what() << "\n";
        return 1;
    }
    std::cout << "loaded " << masterPlan << ": " << planRepository.getPlans().size() << " plans, " << planRepository.getBehaviours().size() << " behaviours, "
              << planRepository.getPlanTypes().size() << " plan types, " << planRepository.getTransitionConditions().size() << " conditions, "
              << planRepository.getTasks().size() << " tasks, " << planRepository.getRoles().size() << " roles\n";
    return 0;
}
