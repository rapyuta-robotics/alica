/*
 * Variable.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef VARIABLE_H_
#define VARIABLE_H_


#include <memory>
#include <string>
#include <sstream>

#include "AlicaElement.h"
#include "engine/constraintmodul/SolverVariable.h"

using namespace std;
namespace alica
{
	/**
	 * A variable is constraint by conditions, feasible values can be queried using a ConstraintQuery.
	 */
	class Variable : public AlicaElement
	{
	public:
		Variable();
		Variable(shared_ptr<SolverVariable> v);
		Variable(long id, string name, string type);
		virtual ~Variable();

		string toString();

		string getType();
		void setType(string type);
		shared_ptr<SolverVariable> getSolverVar();
		void setSolverVar(shared_ptr<SolverVariable> solverVar);


	private:
		string type;

	protected:
		shared_ptr<SolverVariable> solverVar;
	};

} /* namespace Alica */

#endif /* VARIABLE_H_ */
