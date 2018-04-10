<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1479556074049" name="QueryPlan1" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1479556084493" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <quantifiers xsi:type="alica:ForallAgents" id="1479556181307" name="MISSING_NAME" comment="" scope="1479556074050">
      <sorts>X</sorts>
      <sorts>Y</sorts>
      <sorts>Z</sorts>
    </quantifiers>
    <vars>#1479556220234</vars>
    <vars>#1479556572534</vars>
  </conditions>
  <vars id="1479556220234" name="QP1X" comment="" Type=""/>
  <vars id="1479556572534" name="QP1Y" comment="" Type=""/>
  <states id="1479556074050" name="QueryState1" comment="" entryPoint="1479556074051">
    <parametrisation id="1479557255352" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:BehaviourConfiguration">QueryBehaviour1.beh#1479556115746</subplan>
      <subvar>QueryBehaviour1.beh#1479556246733</subvar>
      <var>#1479556220234</var>
    </parametrisation>
    <parametrisation id="1479557270121" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:BehaviourConfiguration">QueryBehaviour1.beh#1479556115746</subplan>
      <subvar>QueryBehaviour1.beh#1479557263650</subvar>
      <var>#1479556572534</var>
    </parametrisation>
    <plans xsi:type="alica:BehaviourConfiguration">QueryBehaviour1.beh#1479556115746</plans>
  </states>
  <entryPoints id="1479556074051" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1479556074050</state>
  </entryPoints>
</alica:Plan>
