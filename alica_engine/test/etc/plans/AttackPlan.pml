<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1402488634525" name="AttackPlan" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1402488646220" name="Attack" comment="" entryPoint="1402488646221">
    <plans xsi:type="alica:Plan">Tackle.pml#1402489318663</plans>
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/AttackOpp.beh#1402489366699</plans>
    <inTransitions>#1402489460694</inTransitions>
    <outTransitions>#1402489459382</outTransitions>
  </states>
  <states id="1402489396914" name="Shoot" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/Attack.beh#1402488866727</plans>
    <inTransitions>#1402489459382</inTransitions>
    <outTransitions>#1402489460694</outTransitions>
  </states>
  <transitions id="1402489459382" name="" comment="" msg="">
    <preCondition id="1402489460549" name="" comment="" conditionString="" pluginName="" enabled="true"/>
    <inState>#1402488646220</inState>
    <outState>#1402489396914</outState>
  </transitions>
  <transitions id="1402489460694" name="" comment="" msg="">
    <preCondition id="1402489462088" name="" comment="" conditionString="" pluginName="" enabled="true"/>
    <inState>#1402489396914</inState>
    <outState>#1402488646220</outState>
  </transitions>
  <entryPoints id="1402488646221" name="" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1402488646220</state>
  </entryPoints>
</alica:Plan>
