/*
 * RuleBook.cpp
 *
 *  Created on: Jun 17, 2014
 *      Author: Paul Panin
 */

#include "engine/rules/RuleBook.h"
#include "engine/AlicaEngine.h"

namespace alica
{
	RuleBook::RuleBook()
	{
		this->to = AlicaEngine::getInstance()->getTeamObserver();
//		this->ps = AlicaEngine::getInstance().
	}

	RuleBook::~RuleBook()
	{
		// TODO Auto-generated destructor stub
	}





	/** Getter and Setter **/
	bool RuleBook::isChangeOccured() const
	{
		return changeOccured;
	}

	void RuleBook::setChangeOccured(bool changeOccured)
	{
		this->changeOccured = changeOccured;
	}
}
