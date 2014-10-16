<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1413200862180" name="MultiAgentTestPlan" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" priority="0.0" minCardinality="2" maxCardinality="2">
  <states id="1413200877336" name="OtherState" comment="" entryPoint="1413200877337">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/Attack.beh#1402488866727</plans>
    <outTransitions>#1413201368286</outTransitions>
  </states>
  <states id="1413200910490" name="State1" comment="" entryPoint="1413200890537">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/Attack.beh#1402488866727</plans>
    <outTransitions>#1413201050743</outTransitions>
  </states>
  <states id="1413201030936" name="State2" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/Attack.beh#1402488866727</plans>
    <inTransitions>#1413201050743</inTransitions>
    <outTransitions>#1413201367062</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1413201164999" name="NewSuccessState" comment="">
    <inTransitions>#1413201367062</inTransitions>
    <inTransitions>#1413201368286</inTransitions>
  </states>
  <transitions id="1413201050743" name="" comment="" msg="">
    <preCondition id="1413201052549" name="" comment="" conditionString="" pluginName="" enabled="true"/>
    <inState>#1413200910490</inState>
    <outState>#1413201030936</outState>
  </transitions>
  <transitions id="1413201367062" name="" comment="" msg="">
    <preCondition id="1413201367990" name="" comment="" conditionString="" pluginName="" enabled="true"/>
    <inState>#1413201030936</inState>
    <outState>#1413201164999</outState>
  </transitions>
  <transitions id="1413201368286" name="" comment="" msg="">
    <preCondition id="1413201370590" name="" comment="" conditionString="" pluginName="" enabled="true"/>
    <inState>#1413200877336</inState>
    <outState>#1413201164999</outState>
  </transitions>
  <entryPoints id="1413200877337" name="AttackTask" comment="" successRequired="true" minCardinality="1" maxCardinality="1">
    <task>../Misc/taskrepository.tsk#1407153522080</task>
    <state>#1413200877336</state>
  </entryPoints>
  <entryPoints id="1413200890537" name="NewEntryPoint" comment="" successRequired="true" minCardinality="1" maxCardinality="1">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1413200910490</state>
  </entryPoints>
</alica:Plan>
