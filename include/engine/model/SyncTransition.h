/*
 * SyncTransition.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef SYNCTRANSITION_H_
#define SYNCTRANSITION_H_

#include "AlicaElement.h"
#include "Plan.h"

namespace alica
{

	/*
	 *
	 */
	class SyncTransition : public alica::AlicaElement
	{
	public:
		SyncTransition();
		virtual ~SyncTransition();
		bool isFailOnSyncTimeOut() const;
		void setFailOnSyncTimeOut(bool failOnSyncTimeOut);
		unsigned long getSyncTimeOut() const;
		void setSyncTimeOut(unsigned long syncTimeOut);
		unsigned long getTalkTimeOut() const;
		void setTalkTimeOut(unsigned long talkTimeOut);
		const Plan* getPlan() const;
		void setPlan(Plan* plan);

	private:
		unsigned long talkTimeOut ;
		unsigned long syncTimeOut;
		bool failOnSyncTimeOut;
		Plan* plan;
	};

} /* namespace Alica */

#endif /* SYNCTRANSITION_H_ */
