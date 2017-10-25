#include "supplementary/AgentIDManager.h"
#include "supplementary/IAgentIDFactory.h"
namespace supplementary{
/**
 * The method for getting the singleton instance.
 * @return A pointer to the AgentIDManager object, you must not delete.
 */
AgentIDManager *AgentIDManager::getInstance()
{
    static AgentIDManager instance;
    return &instance;
}
AgentIDManager::AgentIDManager()
    : idFactory(nullptr)
{
}

AgentIDManager::~AgentIDManager()
{
	for (auto& id : this->agentIDs)
	{
		delete id;
	}
}
}
