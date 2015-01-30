<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1402488893641" name="Defend" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1402488903549" name="Tackle" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/Tackle.beh#1402488956661</plans>
    <plans xsi:type="alica:Plan">Tackle.pml#1402489318663</plans>
    <inTransitions>#1402488990761</inTransitions>
    <outTransitions>#1402488991762</outTransitions>
  </states>
  <states xsi:type="alica:FailureState" id="1402488910751" name="GetGoal" comment="GetGoal">
    <inTransitions>#1402489071510</inTransitions>
  </states>
  <states id="1402488959965" name="GetBall" comment="" entryPoint="1402488903550">
    <inTransitions>#1402488991762</inTransitions>
    <outTransitions>#1402488990761</outTransitions>
    <outTransitions>#1402489064693</outTransitions>
  </states>
  <states id="1402489037735" name="TryToDefendGoal" comment="">
    <plans xsi:type="alica:PlanType">PlanType.pty#1402489564599</plans>
    <inTransitions>#1402489064693</inTransitions>
    <outTransitions>#1402489071510</outTransitions>
  </states>
  <transitions id="1402488990761" name="TackleToGetBall" comment="GetBallToTackle" msg="">
    <preCondition id="1402488991641" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1402488959965</inState>
    <outState>#1402488903549</outState>
  </transitions>
  <transitions id="1402488991762" name="TackleToGetBall" comment="TackleToGetBall" msg="">
    <preCondition id="1402488993122" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1402488903549</inState>
    <outState>#1402488959965</outState>
  </transitions>
  <transitions id="1402489064693" name="GetBallToTryToDefendGoal" comment="TESTESTETS" msg="">
    <preCondition id="1402489065962" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1402488959965</inState>
    <outState>#1402489037735</outState>
  </transitions>
  <transitions id="1402489071510" name="TryToDefendGoalToGetGoal" comment="TryToDefendGoalToGetGoal" msg="">
    <preCondition id="1402489073613" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1402489037735</inState>
    <outState>#1402488910751</outState>
  </transitions>
  <entryPoints id="1402488903550" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1402488959965</state>
  </entryPoints>
</alica:Plan>
