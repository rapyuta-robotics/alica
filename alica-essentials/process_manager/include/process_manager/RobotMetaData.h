#pragma once

#include <string>

namespace supplementary
{
	class IAgentID;

	class RobotMetaData
	{
	public:
		RobotMetaData(std::string name, const IAgentID* agentID);
		virtual ~RobotMetaData();

		const IAgentID* agentID;
		std::string name;
	};
} /* namespace supplementary */
