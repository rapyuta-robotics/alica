/*
 * PlanChange.h
 *
 *  Created on: Jul 15, 2014
 *      Author: snook
 */

#ifndef PLANCHANGE_H_
#define PLANCHANGE_H_

namespace alica
{
	/**
	 * Captures the result of a rule application.
	 */
	enum PlanChange
	{
		NoChange, //!< NoChange occurred, rule was not applicable
		InternalChange, //!< InternalChange, change occurred but is of no interest to upper level plans
		SuccesChange, //!< SuccesChange, change occurred and led to a success, upper level can react
		FailChange //!< FailChange, change occurred and led to a failure, upper level should react
	};
}

#endif /* PLANCHANGE_H_ */
