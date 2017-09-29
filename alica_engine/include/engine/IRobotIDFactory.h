#pragma once

#include <engine/IRobotID.h>

namespace alica{

	class IRobotIDFactory
	{
	public:
		virtual ~IRobotIDFactory() {};
		virtual const IRobotID * create(std::vector<uint8_t > &b) const = 0;

	};

}
