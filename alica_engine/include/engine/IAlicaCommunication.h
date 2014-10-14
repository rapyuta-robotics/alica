/*
 * IAlicaCommunication.h
 *
 *  Created on: Jun 24, 2014
 *      Author: Stephan Opfer
 */

#ifndef IALICACOMMUNICATION_H_
#define IALICACOMMUNICATION_H_

#include "AlicaEngine.h"

namespace alica
{
	class RoleSwitch;
	struct SyncTalk;
	struct SyncReady;
	struct BehaviourEngineInfo;
	struct PlanTreeInfo;
	struct AllocationAuthorityInfo;

	class IAlicaCommunication
	{
	public:
		IAlicaCommunication(AlicaEngine* ae);
		virtual ~IAlicaCommunication(){}

		virtual void sendAllocationAuthority(AllocationAuthorityInfo& aai) = 0;
		virtual void sendBehaviourEngineInfo(BehaviourEngineInfo& bi) = 0;
		virtual void sendPlanTreeInfo(PlanTreeInfo& pti) = 0;
		virtual void sendRoleSwitch(RoleSwitch& rs) = 0;
		virtual void sendSyncReady(SyncReady& sr) = 0;
		virtual void sendSyncTalk(SyncTalk& st) = 0;

		//TODO call
		virtual void tick() {};

		void onSyncTalkReceived(shared_ptr<SyncTalk> st);
		void onSyncReadyReceived(shared_ptr<SyncReady> sr);
		void onAuthorityInfoReceived(shared_ptr<AllocationAuthorityInfo> aai);
		void onPlanTreeInfoReceived(shared_ptr<PlanTreeInfo> pti);

		virtual void startCommunication() = 0;
		virtual void stopCommunication() = 0;

	protected:
		AlicaEngine* ae;
	};

} /* namespace alica */

#endif /* IALICACOMMUNICATION_H_ */
