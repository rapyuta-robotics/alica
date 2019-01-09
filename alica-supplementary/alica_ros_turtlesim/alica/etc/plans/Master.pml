<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1542881176278" name="Master" comment="" masterPlan="true" utilityFunction="MasterUtility" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1542881176280" name="Init" comment="" entryPoint="1543227864154">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/Go2RandomPosition.beh#1542881996304</plans>
    <inTransitions>#1542881648973</inTransitions>
    <outTransitions>#1542881645594</outTransitions>
  </states>
  <states id="1542881580237" name="Move" comment="">
    <plans xsi:type="alica:Plan">Move.pml#1542882005838</plans>
    <inTransitions>#1542881645594</inTransitions>
    <outTransitions>#1542881648973</outTransitions>
  </states>
  <transitions id="1542881645594" name="Init2Move" comment="" msg="">
    <preCondition id="1542881647180" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1542881176280</inState>
    <outState>#1542881580237</outState>
  </transitions>
  <transitions id="1542881648973" name="Move2Init" comment="Transition from Move to Init" msg="">
    <preCondition id="1542881650423" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1542881580237</inState>
    <outState>#1542881176280</outState>
  </transitions>
  <entryPoints id="1543227864154" name="NewEntryPoint" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1542881176318</task>
    <state>#1542881176280</state>
  </entryPoints>
</alica:Plan>
