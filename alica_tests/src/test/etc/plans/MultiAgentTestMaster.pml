<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1413200842973" name="MultiAgentTestMaster" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1413200842974" name="Init" comment="" entryPoint="1413200842975">
    <outTransitions>#1413201226246</outTransitions>
  </states>
  <states id="1413201213955" name="Start" comment="">
    <plans xsi:type="alica:Plan">MultiAgentTestPlan.pml#1413200862180</plans>
    <inTransitions>#1413201226246</inTransitions>
    <outTransitions>#1413201388722</outTransitions>
  </states>
  <states id="1413201380359" name="Finished" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/Attack.beh#1402488866727</plans>
    <inTransitions>#1413201388722</inTransitions>
  </states>
  <transitions id="1413201226246" name="" comment="" msg="">
    <preCondition id="1413201227586" name="" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1413200842974</inState>
    <outState>#1413201213955</outState>
  </transitions>
  <transitions id="1413201388722" name="" comment="" msg="">
    <preCondition id="1413201389955" name="" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1413201213955</inState>
    <outState>#1413201380359</outState>
  </transitions>
  <entryPoints id="1413200842975" name="" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1413200842974</state>
  </entryPoints>
</alica:Plan>
