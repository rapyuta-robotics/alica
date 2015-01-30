<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1418825395939" name="MasterSyncTransition" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="2" maxCardinality="20000">
  <states id="1418825395940" name="FirstTaskFirstState" comment="" entryPoint="1418825395941">
    <outTransitions>#1418825425833</outTransitions>
  </states>
  <states id="1418825404963" name="SecondTaskFirstState" comment="" entryPoint="1418825402617">
    <outTransitions>#1418825427469</outTransitions>
  </states>
  <states id="1418825409988" name="FirstTaskSecondState" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/Attack.beh#1402488866727</plans>
    <inTransitions>#1418825425833</inTransitions>
  </states>
  <states id="1418825411686" name="SecondTaskSecondState" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/Attack.beh#1402488866727</plans>
    <inTransitions>#1418825427469</inTransitions>
  </states>
  <transitions id="1418825425833" name="FirstTaskTran" comment="" msg="">
    <preCondition id="1418825427317" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1418825395940</inState>
    <outState>#1418825409988</outState>
    <synchronisation>#1418825482116</synchronisation>
  </transitions>
  <transitions id="1418825427469" name="SecondTaskTran" comment="" msg="">
    <preCondition id="1418825428924" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1418825404963</inState>
    <outState>#1418825411686</outState>
    <synchronisation>#1418825482116</synchronisation>
  </transitions>
  <synchronisations id="1418825482116" name="Sync" comment="" synchedTransitions="1418825425833 1418825427469" talkTimeout="30" syncTimeout="10000" failOnSyncTimeOut="false"/>
  <entryPoints id="1418825395941" name="AttackTask" comment="" successRequired="false" minCardinality="1" maxCardinality="10000">
    <task>../Misc/taskrepository.tsk#1407153522080</task>
    <state>#1418825395940</state>
  </entryPoints>
  <entryPoints id="1418825402617" name="DefaultTask" comment="" successRequired="false" minCardinality="1" maxCardinality="10000">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1418825404963</state>
  </entryPoints>
</alica:Plan>
