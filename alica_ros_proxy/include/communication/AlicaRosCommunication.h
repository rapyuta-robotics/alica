/*
 * AlicaRosCommunication.h
 *
 *  Created on: 10.09.2014
 *      Author: endy
 */

#ifndef ALICAROSCOMMUNICATION_H_
#define ALICAROSCOMMUNICATION_H_


#include "engine/IAlicaCommunication.h"

using namespace alica;


namespace alicaRosProxy
{

	class AlicaRosCommunication : public alica::IAlicaCommunication
	{
	public:
		AlicaRosCommunication(AlicaEngine* ae);
		virtual ~AlicaRosCommunication();

		virtual void tick();

		virtual void sendAllocationAuthority(AllocationAuthorityInfo aai);
		virtual void sendBehaviourEngineInfo(BehaviourEngineInfo bi);
		virtual void sendPlanTreeInfo(PlanTreeInfo pti);
		virtual void sendRoleSwitch(RoleSwitch rs);
		virtual void sendSyncReady(SyncReady sr);
		virtual void sendSyncTalk(SyncTalk st);
	};

} /* namespace alicaRosProxy */

#endif /* ALICAROSCOMMUNICATION_H_ */
