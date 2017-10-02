#pragma once

#include "IAgentID.h"

namespace supplementary{

	class IAgentIDFactory
	{
	public:
		virtual ~IAgentIDFactory() {};
		virtual const IAgentID * create(std::vector<uint8_t > &b) const = 0;

	};

} /* namespace supplementary */
