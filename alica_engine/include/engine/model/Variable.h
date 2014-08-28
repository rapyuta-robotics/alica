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

namespace AutoDiff
{
	class Variable;
}

namespace alica
{

	class Variable : public AlicaElement
	{
	public:
		Variable();
		Variable(AutoDiff::Variable* v);
		Variable(long id, string name, string type);
		virtual ~Variable();

		string toString();

		string getType();
		void setType(string type);
		AutoDiff::Variable* getSolverVar();


	private:
		string type;
		void setSolverVar(AutoDiff::Variable* solverVar);

	protected:
		AutoDiff::Variable* solverVar;
	};

} /* namespace Alica */

#endif /* VARIABLE_H_ */
