<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1479557378264" name="ProbBuildingLevel1" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/ProblemModule" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <vars id="1479557432793" name="PBL1X" comment="" Type=""/>
  <vars id="1479557444388" name="PBL1Y" comment="" Type=""/>
  <states id="1479557378265" name="PTState" comment="" entryPoint="1479557378266">
    <parametrisation id="1479557505697" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:PlanType">QueryPlantype.pty#1479557395790</subplan>
      <subvar>QueryPlantype.pty#1479557463468</subvar>
      <var>#1479557432793</var>
    </parametrisation>
    <parametrisation id="1479557512341" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:PlanType">QueryPlantype.pty#1479557395790</subplan>
      <subvar>QueryPlantype.pty#1479557473424</subvar>
      <var>#1479557444388</var>
    </parametrisation>
    <plans xsi:type="alica:PlanType">QueryPlantype.pty#1479557395790</plans>
  </states>
  <entryPoints id="1479557378266" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1479557378265</state>
  </entryPoints>
</alica:Plan>
