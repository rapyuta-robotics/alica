#include "alica_capnzero_proxy/ContainerUtils.h"

// Generated CapnProto Messages:
#include "alica_msg/AlicaEngineInfo.capnp.h"
#include "alica_msg/AllocationAuthorityInfo.capnp.h"
#include "alica_msg/PlanTreeInfo.capnp.h"
#include "alica_msg/RoleSwitch.capnp.h"
#include "alica_msg/SolverResult.capnp.h"
#include "alica_msg/SyncReady.capnp.h"
#include "alica_msg/SyncTalk.capnp.h"

#include <essentials/IDManager.h>
#include <essentials/WildcardID.h>

namespace alica_capnzero_proxy
{
alica::AllocationAuthorityInfo ContainerUtils::toAllocationAuthorityInfo(::capnp::FlatArrayMessageReader& msg, essentials::IDManager* idManager)
{
    alica_msgs::AllocationAuthorityInfo::Reader reader = msg.getRoot<alica_msgs::AllocationAuthorityInfo>();
#ifdef CAPNZERO_PROXY_DEBUG
    std::cout << "alica_capnzero_proxy::ContainerUtils: Received '" << reader.toString().flatten().cStr() << "'" << std::endl;
#endif

    alica::AllocationAuthorityInfo aai;
    aai.senderID = idManager->getIDFromBytes(
            reader.getSenderId().getValue().asBytes().begin(), reader.getSenderId().getValue().size(), static_cast<uint8_t>(reader.getSenderId().getType()));
    aai.planId = reader.getPlanId();
    aai.planType = reader.getPlanType();
    aai.parentState = reader.getParentState();
    aai.authority = idManager->getIDFromBytes(
            reader.getAuthority().getValue().asBytes().begin(), reader.getAuthority().getValue().size(), static_cast<uint8_t>(reader.getAuthority().getType()));
    ::capnp::List<alica_msgs::EntrypointRobots>::Reader entryPointRobots = reader.getEntrypointRobots();
    for (unsigned int i = 0; i < entryPointRobots.size(); ++i) {
        aai.entryPointRobots.emplace_back();

        alica_msgs::EntrypointRobots::Reader tmpEntrypointRobot = entryPointRobots[i];
        aai.entryPointRobots[i].entrypoint = tmpEntrypointRobot.getEntrypoint();

        ::capnp::List<capnzero::ID>::Reader robots = tmpEntrypointRobot.getRobots();
        for (unsigned int j = 0; j < robots.size(); ++j) {
            // CapnProto uses uint16_t for enums, but we use uint8_t hopefully it works for us - otherwise we need a matching/translation via switch case
            aai.entryPointRobots[i].robots.push_back(
                    idManager->getIDFromBytes(robots[j].getValue().asBytes().begin(), robots[j].getValue().size(), (uint8_t) robots[j].getType()));
        }
    }

    return aai;
}

alica::AlicaEngineInfo ContainerUtils::toAlicaEngineInfo(::capnp::FlatArrayMessageReader& msg, essentials::IDManager* idManager) {
    alica_msgs::AlicaEngineInfo::Reader reader = msg.getRoot<alica_msgs::AlicaEngineInfo>();
#ifdef CAPNZERO_PROXY_DEBUG
    std::cout << "alica_capnzero_proxy::ContainerUtils: Received '" << reader.toString().flatten().cStr() << "'" << std::endl;
#endif

    alica::AlicaEngineInfo aei;
    aei.senderID = idManager->getIDFromBytes(
            reader.getSenderId().getValue().asBytes().begin(), reader.getSenderId().getValue().size(), static_cast<uint8_t>(reader.getSenderId().getType()));
    aei.currentState = reader.getCurrentState();
    aei.masterPlan = reader.getMasterPlan();
    aei.currentTask = reader.getCurrentTask();
    aei.currentRole = reader.getCurrentRole();
    aei.currentPlan = reader.getCurrentPlan();
    ::capnp::List<capnzero::ID>::Reader agentIdsWithMe = reader.getAgentIdsWithMe();
    for (unsigned int i = 0; i < agentIdsWithMe.size(); ++i) {
        aei.robotIDsWithMe.push_back(idManager->getIDFromBytes(agentIdsWithMe[i].getValue().asBytes().begin(), agentIdsWithMe[i].getValue().size(), (uint8_t) agentIdsWithMe[i].getType()));
    }
    return aei;
}

alica::PlanTreeInfo ContainerUtils::toPlanTreeInfo(::capnp::FlatArrayMessageReader& msg, essentials::IDManager* idManager)
{
    alica_msgs::PlanTreeInfo::Reader reader = msg.getRoot<alica_msgs::PlanTreeInfo>();
#ifdef CAPNZERO_PROXY_DEBUG
    std::cout << "alica_capnzero_proxy::ContainerUtils: Received '" << reader.toString().flatten().cStr() << "'" << std::endl;
#endif

    alica::PlanTreeInfo pti;
    pti.senderID = idManager->getIDFromBytes(
            reader.getSenderId().getValue().asBytes().begin(), reader.getSenderId().getValue().size(), (uint8_t) reader.getSenderId().getType());

    ::capnp::List<int64_t>::Reader states = reader.getStateIds();
    for (unsigned int i = 0; i < states.size(); ++i) {
        pti.stateIDs.push_back(states[i]);
    }

    ::capnp::List<int64_t>::Reader succeded = reader.getSucceededEps();
    for (unsigned int j = 0; j < succeded.size(); ++j) {
        pti.succeededEPs.push_back(succeded[j]);
    }
    return pti;
}

alica::SyncReady ContainerUtils::toSyncReady(capnp::FlatArrayMessageReader& msg, essentials::IDManager* idManager)
{
    alica_msgs::SyncReady::Reader reader = msg.getRoot<alica_msgs::SyncReady>();
#ifdef CAPNZERO_PROXY_DEBUG
    std::cout << "alica_capnzero_proxy::ContainerUtils: Received '" << reader.toString().flatten().cStr() << "'" << std::endl;
#endif
    alica::SyncReady sr;
    sr.senderID = idManager->getIDFromBytes(
            reader.getSenderId().getValue().asBytes().begin(), reader.getSenderId().getValue().size(), (uint8_t) reader.getSenderId().getType());
    sr.synchronisationID = reader.getSynchronisationId();
    return sr;
}

alica::SyncTalk ContainerUtils::toSyncTalk(capnp::FlatArrayMessageReader& msg, essentials::IDManager* idManager)
{
    alica_msgs::SyncTalk::Reader reader = msg.getRoot<alica_msgs::SyncTalk>();
#ifdef CAPNZERO_PROXY_DEBUG
    std::cout << "alica_capnzero_proxy::ContainerUtils: Received '" << reader.toString().flatten().cStr() << "'" << std::endl;
#endif

    alica::SyncTalk st;
    st.senderID = idManager->getIDFromBytes(
            reader.getSenderId().getValue().asBytes().begin(), reader.getSenderId().getValue().size(), (uint8_t) reader.getSenderId().getType());
    capnp::List<alica_msgs::SyncData>::Reader msgSyncData = reader.getSyncData();
    for (unsigned int i = 0; i < msgSyncData.size(); ++i) {
        st.syncData.emplace_back();
        alica_msgs::SyncData::Reader tmpSyncData = msgSyncData[i];
        st.syncData[i].ack = tmpSyncData.getAck();
        st.syncData[i].conditionHolds = tmpSyncData.getTransitionHolds();
        st.syncData[i].transitionID = tmpSyncData.getTransitionId();
        st.syncData[i].robotID = idManager->getIDFromBytes(tmpSyncData.getRobotId().getValue().asBytes().begin(), tmpSyncData.getRobotId().getValue().size(),
                (uint8_t) tmpSyncData.getRobotId().getType());
    }
    return st;
}

alica::SolverResult ContainerUtils::toSolverResult(capnp::FlatArrayMessageReader& msg, essentials::IDManager* idManager)
{
    alica_msgs::SolverResult::Reader reader = msg.getRoot<alica_msgs::SolverResult>();
#ifdef CAPNZERO_PROXY_DEBUG
    std::cout << "alica_capnzero_proxy::ContainerUtils: Received '" << reader.toString().flatten().cStr() << "'" << std::endl;
#endif
    alica::SolverResult solverResult;
    solverResult.senderID = idManager->getIDFromBytes(
            reader.getSenderId().getValue().asBytes().begin(), reader.getSenderId().getValue().size(), (uint8_t) reader.getSenderId().getType());

    capnp::List<alica_msgs::SolverVar>::Reader msgSolverVars = reader.getVars();
    for (unsigned int i = 0; i < msgSolverVars.size(); ++i) {
        alica_msgs::SolverVar::Reader tmpVar = msgSolverVars[i];
        solverResult.vars.emplace_back();
        solverResult.vars[i].id = tmpVar.getId();
        std::vector<uint8_t> tmp;
        capnp::List<uint8_t>::Reader val = tmpVar.getValue();
        for (unsigned int j = 0; j < val.size(); ++j) {
            solverResult.vars[i].value[j] = val[i];
        }
    }
    return solverResult;
}

void ContainerUtils::toMsg(alica::AllocationAuthorityInfo aai, ::capnp::MallocMessageBuilder& msgBuilder)
{
    alica_msgs::AllocationAuthorityInfo::Builder msg = msgBuilder.initRoot<alica_msgs::AllocationAuthorityInfo>();

    msg.setParentState(aai.parentState);
    msg.setPlanId(aai.planId);
    msg.setParentState(aai.parentState);
    msg.setPlanType(aai.planType);
    capnzero::ID::Builder sender = msg.initSenderId();
    sender.setValue(kj::arrayPtr(aai.senderID->getRaw(), (unsigned int) aai.senderID->getSize()));
    sender.setType(aai.senderID->getType());
    capnzero::ID::Builder authority = msg.initAuthority();
    authority.setValue(kj::arrayPtr(aai.authority->getRaw(), (unsigned int) aai.authority->getSize()));
    ::capnp::List<alica_msgs::EntrypointRobots>::Builder entrypoints = msg.initEntrypointRobots((unsigned int) aai.entryPointRobots.size());
    for (unsigned int i = 0; i < aai.entryPointRobots.size(); ++i) {
        auto ep = aai.entryPointRobots[i];
        alica_msgs::EntrypointRobots::Builder tmp = entrypoints[i];
        tmp.setEntrypoint(ep.entrypoint);
        ::capnp::List<capnzero::ID>::Builder tmpRobots = tmp.initRobots((unsigned int) ep.robots.size());
        for (unsigned int j = 0; j < ep.robots.size(); ++i) {
            capnzero::ID::Builder tmpUUID = tmpRobots[j];
            tmpUUID.setValue(kj::arrayPtr(ep.robots[j]->getRaw(), (unsigned int) ep.robots[j]->getSize()));
            tmpUUID.setType(ep.robots[j]->getType());
        }
    }
}

void ContainerUtils::toMsg(alica::AlicaEngineInfo aei, ::capnp::MallocMessageBuilder& msgBuilder)
{
    alica_msgs::AlicaEngineInfo::Builder msg = msgBuilder.initRoot<alica_msgs::AlicaEngineInfo>();

    capnzero::ID::Builder sender = msg.initSenderId();
    sender.setValue(kj::arrayPtr(aei.senderID->getRaw(), (unsigned int) aei.senderID->getSize()));
    sender.setType(aei.senderID->getType());

    msg.setMasterPlan(aei.masterPlan);
    msg.setCurrentPlan(aei.currentPlan);
    msg.setCurrentRole(aei.currentRole);
    msg.setCurrentState(aei.currentState);
    msg.setCurrentTask(aei.currentTask);

    ::capnp::List<capnzero::ID>::Builder agents = msg.initAgentIdsWithMe((unsigned int) aei.robotIDsWithMe.size());
    for (unsigned int i = 0; i < aei.robotIDsWithMe.size(); ++i) {
        auto& robo = aei.robotIDsWithMe[i];
        capnzero::ID::Builder tmp = agents[0];
        tmp.setValue(kj::arrayPtr(robo->getRaw(), (unsigned int) robo->getSize()));
        tmp.setType(robo->getType());
    }
}

void ContainerUtils::toMsg(alica::PlanTreeInfo pti, ::capnp::MallocMessageBuilder& msgBuilder)
{
    alica_msgs::PlanTreeInfo::Builder msg = msgBuilder.initRoot<alica_msgs::PlanTreeInfo>();
    capnzero::ID::Builder sender = msg.initSenderId();
    sender.setValue(kj::arrayPtr(pti.senderID->getRaw(), (unsigned int) pti.senderID->getSize()));
    sender.setType(pti.senderID->getType());
    ::capnp::List<int64_t>::Builder stateIds = msg.initStateIds((unsigned int) pti.stateIDs.size());
    for (unsigned int i = 0; i < pti.stateIDs.size(); ++i) {
        stateIds.set(i, pti.stateIDs[i]);
    }
    ::capnp::List<int64_t>::Builder succededEps = msg.initSucceededEps((unsigned int) pti.succeededEPs.size());
    for (unsigned int i = 0; i < pti.succeededEPs.size(); ++i) {
        succededEps.set(i, pti.succeededEPs[i]);
    }
}

void ContainerUtils::toMsg(alica::RoleSwitch rs, ::capnp::MallocMessageBuilder& msgBuilder)
{
    alica_msgs::RoleSwitch::Builder msg = msgBuilder.initRoot<alica_msgs::RoleSwitch>();
    capnzero::ID::Builder sender = msg.initSenderId();
    sender.setValue(kj::arrayPtr(rs.senderID->getRaw(), (unsigned int) rs.senderID->getSize()));
    sender.setType(rs.senderID->getType());
    msg.setRoleId(rs.roleID);
}

void ContainerUtils::toMsg(alica::SyncReady sr, ::capnp::MallocMessageBuilder& msgBuilder)
{
    alica_msgs::SyncReady::Builder msg = msgBuilder.initRoot<alica_msgs::SyncReady>();
    capnzero::ID::Builder sender = msg.initSenderId();
    sender.setValue(kj::arrayPtr(sr.senderID->getRaw(), (unsigned int) sr.senderID->getSize()));
    sender.setType(sr.senderID->getType());
    msg.setSynchronisationId(sr.synchronisationID);
}

void ContainerUtils::toMsg(alica::SyncTalk st, ::capnp::MallocMessageBuilder& msgBuilder)
{
    alica_msgs::SyncTalk::Builder msg = msgBuilder.initRoot<alica_msgs::SyncTalk>();
    capnzero::ID::Builder sender = msg.initSenderId();
    sender.setValue(kj::arrayPtr(st.senderID->getRaw(), (unsigned int) st.senderID->getSize()));
    sender.setType(st.senderID->getType());

    ::capnp::List<alica_msgs::SyncData>::Builder syncData = msg.initSyncData((unsigned int) st.syncData.size());
    for (unsigned int i = 0; i < st.syncData.size(); ++i) {
        auto& ds = st.syncData[i];
        alica_msgs::SyncData::Builder tmpData = syncData[i];
        capnzero::ID::Builder tmpId = tmpData.initRobotId();
        tmpId.setValue(kj::arrayPtr(ds.robotID->getRaw(), (unsigned int) ds.robotID->getSize()));
        tmpId.setType(ds.robotID->getType());
        tmpData.setAck(ds.ack);
        tmpData.setTransitionHolds(ds.conditionHolds);
        tmpData.setTransitionId(ds.transitionID);
    }
}

void ContainerUtils::toMsg(alica::SolverResult sr, ::capnp::MallocMessageBuilder& msgBuilder)
{
    alica_msgs::SolverResult::Builder msg = msgBuilder.initRoot<alica_msgs::SolverResult>();
    capnzero::ID::Builder sender = msg.initSenderId();
    sender.setValue(kj::arrayPtr(sr.senderID->getRaw(), (unsigned int) sr.senderID->getSize()));
    sender.setType(sr.senderID->getType());
    ::capnp::List<alica_msgs::SolverVar>::Builder vars = msg.initVars((unsigned int) sr.vars.size());
    for (unsigned int i = 0; i < sr.vars.size(); ++i) {
        auto& var = sr.vars[i];
        alica_msgs::SolverVar::Builder tmpVar = vars[i];
        tmpVar.setId(var.id);
        tmpVar.setValue(kj::arrayPtr(var.value, sizeof(var.value)));
    }
}

} // namespace alica_capnzero_proxy