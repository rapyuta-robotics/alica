/*
 * EntryPointRobotPair.h
 *
 *  Created on: Jul 17, 2014
 *      Author: Stefan Jakob
 */

#ifndef ENTRYPOINTROBOTPAIR_H_
#define ENTRYPOINTROBOTPAIR_H_

using namespace std;

#include "engine/model/EntryPoint.h"
#include <memory>

namespace alica
{


	/**
	 * A simple helper class for conflict detection
	 */
	class EntryPointRobotPair
	{
	public:
		EntryPointRobotPair(EntryPoint* ep, int r);
		virtual ~EntryPointRobotPair();
		EntryPoint* getEntryPoint();
		void setEntryPoint(EntryPoint* entryPoint);
		int getRobot();
		void setRobot(int robot);
		static bool equals(std::shared_ptr<EntryPointRobotPair> thisOne, std::shared_ptr<EntryPointRobotPair> other);

	protected:
		EntryPoint* entryPoint;
		int robot;
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
            return eprp.getEntryPoint()->getId() + 10000 * eprp.getRobot();
        }
    };
}
#endif /* ENTRYPOINTROBOTPAIR_H_ */
