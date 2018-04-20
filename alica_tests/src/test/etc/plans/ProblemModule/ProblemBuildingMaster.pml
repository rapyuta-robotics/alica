<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1479556022226" name="ProblemBuildingMaster" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <vars id="1479557337956" name="PBMX" comment="" Type=""/>
  <vars id="1479557345903" name="PBMY" comment="" Type=""/>
  <states id="1479556022227" name="State1" comment="" entryPoint="1479556022228">
    <parametrisation id="1479557551007" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:Plan">ProbBuildingLevel1.pml#1479557378264</subplan>
      <subvar>ProbBuildingLevel1.pml#1479557444388</subvar>
      <var>#1479557345903</var>
    </parametrisation>
    <parametrisation id="1479557555606" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:Plan">ProbBuildingLevel1.pml#1479557378264</subplan>
      <subvar>ProbBuildingLevel1.pml#1479557432793</subvar>
      <var>#1479557337956</var>
    </parametrisation>
    <plans xsi:type="alica:Plan">ProbBuildingLevel1.pml#1479557378264</plans>
    <outTransitions>#1479557591331</outTransitions>
  </states>
  <states id="1479557585252" name="State2" comment="">
    <plans xsi:type="alica:Plan">ProbBuildingLevel1_1.pml#1479557664989</plans>
    <inTransitions>#1479557591331</inTransitions>
  </states>
  <transitions id="1479557591331" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1479557592662" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true">
      <quantifiers xsi:type="alica:ForallAgents" id="1479557619214" name="MISSING_NAME" comment="" scope="1479556022227">
        <sorts>X</sorts>
        <sorts>Y</sorts>
      </quantifiers>
      <vars>#1479557337956</vars>
      <vars>#1479557345903</vars>
    </preCondition>
    <inState>#1479556022227</inState>
    <outState>#1479557585252</outState>
  </transitions>
  <entryPoints id="1479556022228" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1479556022227</state>
  </entryPoints>
</alica:Plan>
