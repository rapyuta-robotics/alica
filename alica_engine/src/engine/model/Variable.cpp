/*
 * Variable.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Variable.h"


namespace alica
{

	Variable::Variable()
	{
		this->solverVar = nullptr;
	}

	Variable::Variable(shared_ptr<SolverVariable> v)
	{
		this->solverVar = v;
	}

	Variable::~Variable()
	{

	}

	Variable::Variable(long id, string name, string type) :
			Variable()
	{
		this->id = id;
		this->name = name;
		this->type = type;
// TODO:		this->solverVar = new autodiff::Variable();
		this->solverVar = nullptr;
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

	shared_ptr<SolverVariable> Variable::getSolverVar()
	{
		return this->solverVar;
	}

	void alica::Variable::setSolverVar(shared_ptr<SolverVariable> solverVar)
	{
		this->solverVar = solverVar;
	}
} /* namespace Alica */


