<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1414403396328" name="AuthorityTestMaster" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1414403396329" name="testState" comment="">
    <plans xsi:type="alica:Plan">AuthorityTest.pml#1414403413451</plans>
    <inTransitions>#1414403840950</inTransitions>
  </states>
  <states id="1414403820806" name="Init" comment="" entryPoint="1414403396331">
    <outTransitions>#1414403840950</outTransitions>
  </states>
  <transitions id="1414403840950" name="" comment="" msg="">
    <preCondition id="1414403842622" name="" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1414403820806</inState>
    <outState>#1414403396329</outState>
  </transitions>
  <entryPoints id="1414403396331" name="" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1414403820806</state>
  </entryPoints>
</alica:Plan>
