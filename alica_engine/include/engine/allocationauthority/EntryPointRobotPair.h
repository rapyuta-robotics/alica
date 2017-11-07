#pragma once

#include <memory>

#include "supplementary/IAgentID.h"
#include "engine/model/EntryPoint.h"

namespace alica
{

	/**
	 * A simple helper class for conflict detection
	 */
	class EntryPointRobotPair
	{
	public:
		EntryPointRobotPair(EntryPoint* ep, const supplementary::IAgentID* r);
		virtual ~EntryPointRobotPair();
		EntryPoint* getEntryPoint();
		void setEntryPoint(EntryPoint* entryPoint);
		const supplementary::IAgentID* getRobot();
		void setRobot(const supplementary::IAgentID* robot);
		static bool equals(std::shared_ptr<EntryPointRobotPair> thisOne, std::shared_ptr<EntryPointRobotPair> other);

	protected:
		EntryPoint* entryPoint;
		const supplementary::IAgentID* robot;
	};

} /* namespace alica */

namespace std
{
    template<>
    struct hash<alica::EntryPointRobotPair>
    {
        typedef alica::EntryPointRobotPair argument_type;
        typedef std::size_t value_type;

        value_type operator()(argument_type & eprp) const
        {
            return std::hash<long int>()(eprp.getEntryPoint()->getId()) + supplementary::IAgentIDHash()(eprp.getRobot());
        }
    };
}
