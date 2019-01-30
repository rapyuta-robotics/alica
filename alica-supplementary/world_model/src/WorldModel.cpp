#include "supplementary/WorldModel.h"

#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/teammanager/TeamManager.h>
#include <essentials/AgentID.h>

namespace supplementary
{
WorldModel::WorldModel()
        : maySendMessages(true)
        , alicaEngine(nullptr)
        , ownID(nullptr)
{
    essentials::SystemConfig& sc = essentials::SystemConfig::getInstance();
    this->maySendMessages = sc["WorldModel"]->get<bool>("WorldModel", "MaySendMessages", NULL);
}

WorldModel::~WorldModel() {}

void WorldModel::init() {}

bool WorldModel::setEngine(alica::AlicaEngine* ae)
{
    std::cout << "WorldModel: SetEngine called!" << std::endl;
    if (this->alicaEngine == nullptr) {
        this->alicaEngine = ae;
        return true;
    } else {
        return false;
    }
}

alica::AlicaEngine* WorldModel::getEngine()
{
    return this->alicaEngine;
}

alica::AlicaTime WorldModel::getTime()
{
    if (this->alicaEngine != nullptr) {
        return this->alicaEngine->getAlicaClock().now();
    } else {
        return alica::AlicaTime::zero();
    }
}

bool WorldModel::isMaySendMessages() const
{
    return this->maySendMessages;
}

void WorldModel::setMaySendMessages(bool maySendMessages)
{
    this->maySendMessages = maySendMessages;
}

/**
 * The AlicaEngine must be set, before this method is called!
 */
const essentials::AgentID* WorldModel::getOwnId()
{
    if (!this->ownID) {
        this->ownID = this->alicaEngine->getTeamManager().getLocalAgentID().get();
    }
    return this->ownID;
}

} // namespace supplementary
