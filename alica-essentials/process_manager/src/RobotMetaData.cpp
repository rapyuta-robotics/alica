#include "process_manager/RobotMetaData.h"

#include <supplementary/IAgentID.h>

namespace supplementary
{

	RobotMetaData::RobotMetaData(std::string name, const IAgentID* agentID) :
		name(name), agentID(agentID)
	{

	}

	RobotMetaData::~RobotMetaData()
	{

	}

} /* namespace supplementary */
