/*
 * ModelFactory.h
 *
 *  Created on: Mar 27, 2014
 *      Author: Stephan Opfer
 */

#ifndef MODELFACTORY_H_
#define MODELFACTORY_H_

class PlanParser;

#include <memory>

#include "../PlanRepository.h"
namespace alica
{


class ModelFactory
{
public:
//	ModelFactory(PlanParser p, PlanRepository rep);
//	ModelFactory();
	ModelFactory(PlanParser* p, std::shared_ptr<PlanRepository> rep);
	virtual ~ModelFactory();

	bool ignoreMasterPlanId;
	bool getIgnoreMasterPlanId();
	void setIgnoreMasterPlanId(bool value);

protected:
	PlanParser *p;
	std::shared_ptr<PlanRepository> rep;
};

} /* namespace Alica */

#endif /* MODELFACTORY_H_ */
