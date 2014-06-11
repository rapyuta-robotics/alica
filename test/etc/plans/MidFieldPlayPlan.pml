<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1402488770050" name="MidFieldPlayPlan" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1402489260911" name="NewRuntimeCondition" comment="Test RC" conditionString="" pluginName=""/>
  <states id="1402488787818" name="Wander" comment="" entryPoint="1402488787819">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/MidFieldStandard.beh#1402488712657</plans>
    <outTransitions>#1402489257607</outTransitions>
    <outTransitions>#1402489276995</outTransitions>
  </states>
  <states id="1402489237914" name="Tackle" comment="">
    <plans xsi:type="alica:Plan">Tackle.pml#1402489318663</plans>
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/Attack.beh#1402488866727</plans>
    <plans xsi:type="alica:Plan">Defend.pml#1402488893641</plans>
    <inTransitions>#1402489257607</inTransitions>
  </states>
  <states id="1402489273401" name="Sync" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/Tackle.beh#1402488956661</plans>
    <inTransitions>#1402489276995</inTransitions>
  </states>
  <transitions id="1402489257607" name="" comment="" msg="">
    <preCondition id="1402489258509" name="" comment="" conditionString="" pluginName="" enabled="true"/>
    <inState>#1402488787818</inState>
    <outState>#1402489237914</outState>
  </transitions>
  <transitions id="1402489276995" name="" comment="" msg="">
    <preCondition id="1402489278408" name="" comment="" conditionString="" pluginName="" enabled="true"/>
    <inState>#1402488787818</inState>
    <outState>#1402489273401</outState>
  </transitions>
  <entryPoints id="1402488787819" name="" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1402488787818</state>
  </entryPoints>
</alica:Plan>
