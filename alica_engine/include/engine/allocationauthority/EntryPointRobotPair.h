/*
 * EntryPointRobotPair.h
 *
 *  Created on: Jul 17, 2014
 *      Author: Stefan Jakob
 */

#ifndef ENTRYPOINTROBOTPAIR_H_
#define ENTRYPOINTROBOTPAIR_H_

#include <memory>

#include "engine/IRobotID.h"
#include "engine/model/EntryPoint.h"

using namespace std;
namespace alica
{


	/**
	 * A simple helper class for conflict detection
	 */
	class EntryPointRobotPair
	{
	public:
		EntryPointRobotPair(EntryPoint* ep, alica::IRobotID r);
		virtual ~EntryPointRobotPair();
		EntryPoint* getEntryPoint();
		void setEntryPoint(EntryPoint* entryPoint);
		alica::IRobotID getRobot();
		void setRobot(alica::IRobotID robot);
		static bool equals(std::shared_ptr<EntryPointRobotPair> thisOne, std::shared_ptr<EntryPointRobotPair> other);

	protected:
		EntryPoint* entryPoint;
		alica::IRobotID robot;
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
            return std::hash<long int>()(eprp.getEntryPoint()->getId()) + std::hash<alica::IRobotID>()(eprp.getRobot());
        }
    };
}
#endif /* ENTRYPOINTROBOTPAIR_H_ */
