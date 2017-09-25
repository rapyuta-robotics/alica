#include "engine/model/Variable.h"

namespace alica
{

	Variable::Variable()
	{
		this->solverVar = nullptr;
	}

	Variable::Variable(std::shared_ptr<SolverVariable> v)
	{
		this->solverVar = v;
	}

	Variable::~Variable()
	{

	}

	Variable::Variable(long id, std::string name, std::string type) :
			Variable()
	{
		this->id = id;
		this->name = name;
		this->type = type;
		this->solverVar = nullptr;
	}

	std::string Variable::toString()
	{
		stringstream ss;
		ss << "[Variable: Name=" << name << " Id=" << id << endl;
		return ss.str();
	}

//============= Getter and Setter ======================

	std::string Variable::getType()
	{
		return type;
	}

	void Variable::setType(std::string type)
	{
		this->type = type;
	}

	std::shared_ptr<SolverVariable> Variable::getSolverVar()
	{
		return this->solverVar;
	}

	void alica::Variable::setSolverVar(std::shared_ptr<SolverVariable> solverVar)
	{
		this->solverVar = solverVar;
	}
} /* namespace Alica */


