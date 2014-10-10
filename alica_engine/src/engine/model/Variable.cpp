/*
 * Variable.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Variable.h"
#include "Variable.h"

namespace alica
{

	Variable::Variable()
	{
		this->solverVar = nullptr;
	}

	Variable::Variable(autodiff::Variable* v)
	{
		this->solverVar = v;
	}

	Variable::~Variable()
	{
		delete this->solverVar;
	}

	Variable::Variable(long id, string name, string type) :
			Variable()
	{
		this->id = id;
		this->name = name;
		this->type = type;
		this->solverVar = new autodiff::Variable();
	}

	string Variable::toString()
	{
		stringstream ss;
		ss << "[Variable: Name=" << name << " Id=" << id << endl;
		return ss.str();
	}

//============= Getter and Setter ======================

	string Variable::getType()
	{
		return type;
	}

	void Variable::setType(string type)
	{
		this->type = type;
	}

	autodiff::Variable* Variable::getSolverVar()
	{
		return solverVar;
	}

	void alica::Variable::setSolverVar(autodiff::Variable* solverVar)
	{
		this->solverVar = solverVar;
	}
} /* namespace Alica */


