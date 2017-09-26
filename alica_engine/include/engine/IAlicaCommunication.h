#pragma once

#include "AlicaEngine.h"

namespace alica
{
	struct SolverResult;
	class RoleSwitch;
	struct SyncTalk;
	struct SyncReady;
	struct AlicaEngineInfo;
	struct PlanTreeInfo;
	struct AllocationAuthorityInfo;

	class IAlicaCommunication
	{
	public:
		IAlicaCommunication(AlicaEngine* ae);
		virtual ~IAlicaCommunication(){}

		virtual void sendAllocationAuthority(AllocationAuthorityInfo& aai) const = 0;
		virtual void sendAlicaEngineInfo(AlicaEngineInfo& bi) const = 0;
		virtual void sendPlanTreeInfo(PlanTreeInfo& pti) const = 0;
		virtual void sendRoleSwitch(RoleSwitch& rs) const = 0;
		virtual void sendSyncReady(SyncReady& sr) const = 0;
		virtual void sendSyncTalk(SyncTalk& st) const = 0;
		virtual void sendSolverResult(SolverResult& sr) const = 0;
		virtual void sendLogMessage(int level, string& message) const {};

		virtual void tick() {};

		void onSyncTalkReceived(shared_ptr<SyncTalk> st);
		void onSyncReadyReceived(shared_ptr<SyncReady> sr);
		void onAuthorityInfoReceived(shared_ptr<AllocationAuthorityInfo> aai);
		void onPlanTreeInfoReceived(shared_ptr<PlanTreeInfo> pti);
		void onSolverResult(shared_ptr<SolverResult> sr);

		virtual void startCommunication() = 0;
		virtual void stopCommunication() = 0;

	protected:
		AlicaEngine* ae;
	};

} /* namespace alica */
