/*
 * External.cpp
 *
 *  Created on: Aug 6, 2014
 *      Author: sni
 */

#include "External.h"

#include "../libgringo/gringo/value.hh"
#include "ClingWrapper.h"

namespace supplementary
{

External::External(ClingWrapper *clingWrapper, std::shared_ptr<Gringo::Value> gringoValue)
{
  this->clingWrapper = clingWrapper;
  this->gringoValue = gringoValue;
  this->value = Gringo::Output::ExternalType::E_FREE;
}

External::~External()
{
  this->release();
}

void External::release()
{
  this->clingWrapper->releaseExternal(this->gringoValue);
  this->value = Gringo::Output::ExternalType::E_FREE;
}

void External::assign(bool value)
{
  this->clingWrapper->assignExternal(this->gringoValue, value);

  if (value)
  {
    this->value = Gringo::Output::ExternalType::E_TRUE;
  }
  else
  {
    this->value = Gringo::Output::ExternalType::E_FALSE;
  }
}

bool External::isAssigned()
{
  return this->value != Gringo::Output::ExternalType::E_FREE;
}

bool External::isReleased()
{
  return this->value == Gringo::Output::ExternalType::E_FREE;
}

Gringo::Output::ExternalType External::getValue()
{
  return this->value;
}

}
