#include "supplementary/WorldModel.h"

#include <engine/AlicaEngine.h>
#include <engine/teammanager/TeamManager.h>
#include <engine/IAlicaClock.h>
#include <supplementary/AgentID.h>

namespace supplementary
{
WorldModel::WorldModel()
    : maySendMessages(true)
    , alicaEngine(nullptr)
	, ownID(nullptr)
{
	this->sc = supplementary::SystemConfig::getInstance();
    this->maySendMessages = (*this->sc)["WorldModel"]->get<bool>("WorldModel", "MaySendMessages", NULL);
}

WorldModel::~WorldModel()
{
}

bool WorldModel::setEngine(alica::AlicaEngine *ae)
{
	std::cout << "WorldModel: SetEngine called!" << std::endl;
    if (this->alicaEngine == nullptr)
    {
        this->alicaEngine = ae;
        return true;
    }
    else
    {
        return false;
    }
}

alica::AlicaEngine *WorldModel::getEngine()
{
    return this->alicaEngine;
}

InfoTime WorldModel::getTime()
{
    if (this->alicaEngine != nullptr)
    {
        return this->alicaEngine->getIAlicaClock()->now();
    }
    else
    {
        return 0;
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
const supplementary::AgentID* WorldModel::getOwnId()
{
	if (!this->ownID)
	{
		this->ownID = this->alicaEngine->getTeamManager()->getLocalAgentID();
	}
    return this->ownID;
}

supplementary::SystemConfig *WorldModel::getSystemConfig()
{
    return this->sc;
}
}
