/*
 * Log.h
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#ifndef LOG_H_
#define LOG_H_

#include "Term.h"

namespace AutoDiff
{

	class Log : public Term
	{
	public:
		Log(shared_ptr<Term> arg);

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);

		const shared_ptr<Term> getArg();

	private:
		shared_ptr<Term> _arg;
	};

} /* namespace AutoDiff */

#endif /* LOG_H_ */
