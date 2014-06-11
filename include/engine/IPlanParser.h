/*
 * IPlanParser.h
 *
 *  Created on: May 28, 2014
 *      Author: snook
 */

#ifndef IPLANPARSER_H_
#define IPLANPARSER_H_

using namespace std;


#include <stdio.h>
#include <map>
#include <memory>

namespace alica
{
	class Plan;
	class RoleSet;
	class AlicaElement;

	class IPlanParser
	{
	public:
		virtual ~IPlanParser() {}
		virtual Plan* ParsePlanTree(string masterplan) = 0;
		virtual shared_ptr<RoleSet> ParseRoleSet(string roleSetName, string roleSetDir) = 0;
		virtual void IgnoreMasterPlanId(bool val) = 0;
		virtual shared_ptr<map<long, AlicaElement> > GetParsedElements() = 0;
	};
}
#endif /* IPLANPARSER_H_ */
