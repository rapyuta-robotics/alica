/*
 * Variable.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef VARIABLE_H_
#define VARIABLE_H_

using namespace std;

#include <string>
#include <sstream>

#include "AlicaElement.h"

namespace autodiff
{
	class Variable;
}

namespace alica
{

	/**
	 * A variable is constraint by conditions, feasible values can be queried using a ConstraintQuery.
	 */
	class Variable : public AlicaElement
	{
	public:
		Variable();
		Variable(autodiff::Variable* v);
		Variable(long id, string name, string type);
		virtual ~Variable();

		string toString();

		string getType();
		void setType(string type);
		autodiff::Variable* getSolverVar();


	private:
		string type;
		void setSolverVar(autodiff::Variable* solverVar);

	protected:
		autodiff::Variable* solverVar;
	};

} /* namespace Alica */

#endif /* VARIABLE_H_ */
