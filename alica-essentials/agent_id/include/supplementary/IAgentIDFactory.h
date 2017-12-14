#pragma once

#include "IAgentID.h"

namespace supplementary{

	class IAgentIDFactory
	{
	public:
		virtual ~IAgentIDFactory() {};
		virtual const IAgentID* create(const std::vector<uint8_t > &b) const = 0;
		virtual const IAgentID* generateID() const = 0;
		virtual uint8_t getType() const = 0;
	};

} /* namespace supplementary */
