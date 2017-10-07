#pragma once

#include <memory>
#include <string>

namespace alica
{
	struct AlicaEngineInfo;
	struct AllocationAuthorityInfo;
	struct PlanTreeInfo;
	struct SolverResult;
	struct SyncTalk;
	struct SyncReady;
	class AlicaEngine;
	class RoleSwitch;

	class IAlicaCommunication
	{
	public:
		IAlicaCommunication(AlicaEngine* ae):ae(ae) {};
		virtual ~IAlicaCommunication(){}

		virtual void sendAllocationAuthority(AllocationAuthorityInfo& aai) const = 0;
		virtual void sendAlicaEngineInfo(AlicaEngineInfo& bi) const = 0;
		virtual void sendPlanTreeInfo(PlanTreeInfo& pti) const = 0;
		virtual void sendRoleSwitch(RoleSwitch& rs) const = 0;
		virtual void sendSyncReady(SyncReady& sr) const = 0;
		virtual void sendSyncTalk(SyncTalk& st) const = 0;
		virtual void sendSolverResult(SolverResult& sr) const = 0;
		virtual void sendLogMessage(int level, std::string& message) const {};

		virtual void tick() {};

		void onSyncTalkReceived(std::shared_ptr<SyncTalk> st);
		void onSyncReadyReceived(std::shared_ptr<SyncReady> sr);
		void onAuthorityInfoReceived(std::shared_ptr<AllocationAuthorityInfo> aai);
		void onPlanTreeInfoReceived(std::shared_ptr<PlanTreeInfo> pti);
		void onSolverResult(std::shared_ptr<SolverResult> sr);

		virtual void startCommunication() = 0;
		virtual void stopCommunication() = 0;

	protected:
		AlicaEngine* ae;
	};

} /* namespace alica */
