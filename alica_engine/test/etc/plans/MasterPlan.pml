<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1402488437260" name="MasterPlan" comment="comment" masterPlan="true" utilityFunction="" utilityThreshold="0.1" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1402488437261" name="Attack" comment="" entryPoint="1402488437263">
    <plans xsi:type="alica:Plan">AttackPlan.pml#1402488634525</plans>
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/Attack.beh#1402488866727</plans>
    <outTransitions>#1402488517667</outTransitions>
  </states>
  <states id="1402488463437" name="Defend" comment="" entryPoint="1402488484084">
    <plans xsi:type="alica:Plan">Defend.pml#1402488893641</plans>
  </states>
  <states id="1402488470615" name="Goal" comment="">
    <plans xsi:type="alica:Plan">GoalPlan.pml#1402488870347</plans>
    <inTransitions>#1402488519757</inTransitions>
    <outTransitions>#1402488557864</outTransitions>
  </states>
  <states id="1402488477650" name="MidField" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/MidFieldStandard.beh#1402488712657</plans>
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/DefendMid.beh#1402488763903</plans>
    <plans xsi:type="alica:Plan">MidFieldPlayPlan.pml#1402488770050</plans>
    <inTransitions>#1402488517667</inTransitions>
    <outTransitions>#1402488519757</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1402488536570" name="SucGoalState" comment="">
    <inTransitions>#1402488557864</inTransitions>
  </states>
  <transitions id="1402488517667" name="AttackToGoal" comment="AttackToGoal" msg="">
    <preCondition id="1402488519140" name="" comment="" conditionString="" pluginName="" enabled="true"/>
    <inState>#1402488437261</inState>
    <outState>#1402488477650</outState>
  </transitions>
  <transitions id="1402488519757" name="MidFieldToGoal" comment="MidFieldToGoal" msg="">
    <preCondition id="1402488520968" name="" comment="" conditionString="" pluginName="" enabled="true"/>
    <inState>#1402488477650</inState>
    <outState>#1402488470615</outState>
  </transitions>
  <transitions id="1402488557864" name="GoalToSucGoal" comment="GoalToSucGoal" msg="">
    <preCondition id="1402488558741" name="" comment="" conditionString="" pluginName="" enabled="true"/>
    <inState>#1402488470615</inState>
    <outState>#1402488536570</outState>
  </transitions>
  <entryPoints id="1402488437263" name="" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1402488437261</state>
  </entryPoints>
  <entryPoints id="1402488484084" name="" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1402488486725</task>
    <state>#1402488463437</state>
  </entryPoints>
</alica:Plan>
