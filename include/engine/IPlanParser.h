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
#include <memory>

#include "model/Plan.h"
#include "model/RoleSet.h"
#include "model/AlicaElement.h"

namespace alica
{
class IPlanParser
{
public:
	IPlanParser();
	virtual ~IPlanParser() {}
	virtual std::shared_ptr<Plan> ParsePlanTree(std::string masterplan) = 0;
	virtual std::shared_ptr<RoleSet> ParseRoleSet(std::string roleSetName, std::string roleSetDir) = 0;
	virtual void IgnoreMasterPlanId(bool val) = 0;

	virtual std::shared_ptr<std::map<long, alica::AlicaElement> > GetParsedElements() = 0;
};
}
#endif /* IPLANPARSER_H_ */
