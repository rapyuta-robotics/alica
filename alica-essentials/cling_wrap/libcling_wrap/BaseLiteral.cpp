/*
 * BaseLiteral.cpp
 *
 *  Created on: Aug 8, 2014
 *      Author: sni
 */

#include "BaseLiteral.h"

#include "ClingWrapper.h"

namespace supplementary
{

	BaseLiteral::BaseLiteral(ClingWrapper* clingWrapper, const Gringo::Value query, LiteralType type,
								LiteralUpdateType updateType) :
			query(query)
	{
		this->clingWrapper = clingWrapper;
		this->type = type;
		this->updateType = updateType;

		stringstream ss;
		query.print(ss);
		this->queryString = ss.str();
	}

	BaseLiteral::~BaseLiteral()
	{
		//
	}

	bool BaseLiteral::match(const Gringo::Value& value)
	{
		if (value.type() != Gringo::Value::Type::FUNC)
			return false;

		if (this->query.name() != value.name())
			return false;

		if (this->query.args().size() != value.args().size())
			return false;

		for (uint i = 0; i < this->query.args().size(); ++i)
		{
			Gringo::Value arg = this->query.args()[i];

			if (arg.type() == Gringo::Value::Type::STRING && arg.name() == "?")
				continue;

			if (arg != value.args()[i])
				return false;
		}

		return true;
	}

	LiteralType BaseLiteral::getType()
	{
		return this->type;
	}

	const Gringo::Value BaseLiteral::getQuery()
	{
		return this->query;
	}

	const std::string BaseLiteral::getQueryString()
	{
		return this->queryString;
	}

	LiteralUpdateType BaseLiteral::getUpdateType()
	{
		return this->updateType;
	}

	void BaseLiteral::setUpdateType(LiteralUpdateType updateType)
	{
		this->updateType = updateType;
	}

	bool BaseLiteral::getCheckNewLiterals()
	{
		return false;
	}

} /* namespace supplemantary */
