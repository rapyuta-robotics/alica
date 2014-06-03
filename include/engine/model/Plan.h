/*
 * Plan.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef PLAN_H_
#define PLAN_H_

using namespace std;
#include <string>
#include "AbstractPlan.h"

namespace alica
{

	class Plan : public AbstractPlan
	{
	public:
		Plan(long id);
		virtual ~Plan();
		void setFilename (string filename);
		string getFilename();

		virtual string toString() const;

	protected:
		string filename;

	};

} /* namespace Alica */

#endif /* PLAN_H_ */
