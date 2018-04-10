<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1418042656594" name="MasterPlanTestConditionPlanType" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1418042656595" name="Start" comment="" entryPoint="1418042656596">
    <outTransitions>#1418042682960</outTransitions>
  </states>
  <states id="1418042674811" name="Plantype" comment="">
    <plans xsi:type="alica:PlanType">TestPlanType.pty#1418042702402</plans>
    <inTransitions>#1418042682960</inTransitions>
  </states>
  <transitions id="1418042682960" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1418042683692" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1418042656595</inState>
    <outState>#1418042674811</outState>
  </transitions>
  <entryPoints id="1418042656596" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1418042656595</state>
  </entryPoints>
</alica:Plan>
