/*
 * BoolLiteral.cpp
 *
 *  Created on: Aug 8, 2014
 *      Author: sni
 */

#include "BoolLiteral.h"

#include <iostream>

#include "ClingWrapper.h"
#include "clasp/solver.h"

namespace supplementary
{

BoolLiteral::BoolLiteral(ClingWrapper* clingWrapper, const Gringo::Value query, LiteralUpdateType updateType) :
    BaseLiteral(clingWrapper, query, LiteralType::BOOL, updateType)
{
  this->value = false;
  this->literal = 0;
}

BoolLiteral::~BoolLiteral()
{
  //
}

void BoolLiteral::updateOnModel(const Clasp::Solver *s, const Clasp::Model *model)
{
  if (this->literal == 0 || model == nullptr || s == nullptr)
    return;

  this->value = model->isTrue(s->symbolTable()[this->literal].lit);
}

void BoolLiteral::check(unsigned int literal, const Gringo::Value& value)
{
  if (this->match(value))
    this->literal = literal;
}

bool BoolLiteral::getValue()
{
  if (this->updateType == LiteralUpdateType::POLL)
  {
    this->updateOnModel(this->clingWrapper->getLastSolver(), this->clingWrapper->getLastModel());
  }

  return this->value;
}

bool BoolLiteral::getCheckNewLiterals()
{
  return (this->literal == 0);
}

} /* namespace supplementary */
