#pragma once

#include "AlicaElement.h"
#include "engine/constraintmodul/SolverVariable.h"

#include <memory>
#include <string>

namespace alica
{
	/**
	 * A variable is constraint by conditions, feasible values can be queried using a ConstraintQuery.
	 */
	class Variable : public AlicaElement
	{
	public:
		Variable();
		Variable(std::shared_ptr<SolverVariable> v);
		Variable(long id, std::string name, std::string type);
		virtual ~Variable();

		std::string toString();

		std::string getType();
		void setType(std::string type);
		std::shared_ptr<SolverVariable> getSolverVar();
		void setSolverVar(std::shared_ptr<SolverVariable> solverVar);

	private:
		std::string type;

	protected:
		std::shared_ptr<SolverVariable> solverVar;
	};

} /* namespace alica */
