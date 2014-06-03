/*
 * AbstractPlan.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef ABSTRACTPLAN_H_
#define ABSTRACTPLAN_H_

using namespace std;

#include "AlicaElement.h"

namespace alica
{

	class AbstractPlan : public AlicaElement
	{
	public:
		AbstractPlan();
		virtual ~AbstractPlan();
		bool isMasterPlan() const;
		void setMasterPlan(bool isMasterPlan);

		virtual string toString() const;

	protected:
		bool masterPlan;
	};

} /* namespace Alica */

#endif /* ABSTRACTPLAN_H_ */
