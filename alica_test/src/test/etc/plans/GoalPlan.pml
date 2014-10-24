<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1402488870347" name="GoalPlan" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <conditions xsi:type="alica:PreCondition" id="1402489131988" name="PreCondition" comment="Test PC" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
  <conditions xsi:type="alica:RuntimeCondition" id="1403773741874" name="NewRuntimeCondition" comment="" conditionString="test" pluginName="DefaultPlugin">
    <quantifiers xsi:type="alica:ForallAgents" id="1403773772633" name="MISSING_NAME" comment="" scope="1402489152217">
      <sorts>test</sorts>
    </quantifiers>
    <vars>#1403773747758</vars>
  </conditions>
  <vars id="1403773747758" name="test" comment="test" Type="test"/>
  <states id="1402488881799" name="Shoot" comment="" entryPoint="1402488881800">
    <inTransitions>#1402489205153</inTransitions>
    <outTransitions>#1402489173167</outTransitions>
  </states>
  <states id="1402489152217" name="Miss" comment="">
    <inTransitions>#1402489173167</inTransitions>
    <outTransitions>#1402489205153</outTransitions>
    <outTransitions>#1402489216617</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1402489192198" name="Scored" comment="">
    <inTransitions>#1402489216617</inTransitions>
    <postCondition id="1402489620773" name="MISSING_NAME" comment="Test POSTC" conditionString="" pluginName="DefaultPlugin"/>
  </states>
  <transitions id="1402489173167" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1402489174338" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1402488881799</inState>
    <outState>#1402489152217</outState>
  </transitions>
  <transitions id="1402489205153" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1402489206278" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1402489152217</inState>
    <outState>#1402488881799</outState>
  </transitions>
  <transitions id="1402489216617" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1402489218027" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1402489152217</inState>
    <outState>#1402489192198</outState>
  </transitions>
  <entryPoints id="1402488881800" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1402488881799</state>
  </entryPoints>
</alica:Plan>
