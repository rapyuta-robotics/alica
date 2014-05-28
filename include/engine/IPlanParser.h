/*
 * IPlanParser.h
 *
 *  Created on: May 28, 2014
 *      Author: snook
 */

#ifndef IPLANPARSER_H_
#define IPLANPARSER_H_

#include <stdio.h>
#include <map>

#include "Plan.h"
#include "RoleSet.h"
#include "AlicaElement.h"

namespace alica
{
class IPlanParser
{
public:
	IPlanParser();
	virtual ~IPlanParser()
	{
	}
	virtual alica::Plan ParsePlanTree(std::string masterplan) = 0;
	virtual alica::RoleSet ParseRoleSet(std::string roleSetName, std::string roleSetDir) = 0;
	virtual void IgnoreMasterPlanId(bool val) = 0;

	virtual std::map<long, alica::AlicaElement> GetParsedElements() = 0;
};
}
#endif /* IPLANPARSER_H_ */
