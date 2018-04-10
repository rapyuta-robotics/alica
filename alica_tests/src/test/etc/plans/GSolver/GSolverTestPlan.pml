<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1417423757243" name="GSolverTestPlan" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/GSolver" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1417424512343" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <vars>#1417444589341</vars>
    <vars>#1417444593509</vars>
  </conditions>
  <vars id="1417444589341" name="X" comment="" Type=""/>
  <vars id="1417444593509" name="Y" comment="" Type=""/>
  <states id="1417423777544" name="SolverState" comment="" entryPoint="1417423777546">
    <parametrisation id="1417444715623" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:BehaviourConfiguration">SolverTestBehaviour.beh#1417424483320</subplan>
      <subvar>SolverTestBehaviour.beh#1417444710642</subvar>
      <var>#1417444593509</var>
    </parametrisation>
    <parametrisation id="1417444720874" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:BehaviourConfiguration">SolverTestBehaviour.beh#1417424483320</subplan>
      <subvar>SolverTestBehaviour.beh#1417444707321</subvar>
      <var>#1417444589341</var>
    </parametrisation>
    <plans xsi:type="alica:BehaviourConfiguration">SolverTestBehaviour.beh#1417424483320</plans>
  </states>
  <entryPoints id="1417423777546" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1417423777544</state>
  </entryPoints>
</alica:Plan>
