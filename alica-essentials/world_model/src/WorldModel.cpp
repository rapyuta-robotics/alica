#include "supplementary/WorldModel.h"

#include <engine/AlicaEngine.h>
#include <engine/IAlicaClock.h>

namespace supplementary
{
WorldModel::WorldModel()
    : maySendMessages(true)
    , alicaEngine(nullptr)
{
    this->ownID = supplementary::SystemConfig::getOwnRobotID();

    this->sc = supplementary::SystemConfig::getInstance();
    this->maySendMessages = (*this->sc)["WorldModel"]->get<bool>("WorldModel", "MaySendMessages", NULL);
}

WorldModel::~WorldModel()
{
}

bool WorldModel::setEngine(alica::AlicaEngine *ae)
{
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

int WorldModel::getOwnId()
{
    return this->ownID;
}

supplementary::SystemConfig *WorldModel::getSystemConfig()
{
    return this->sc;
}
}
