<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1530004915640" name="HandleFailExplicit" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1530004915641" name="A" comment="" entryPoint="1530004915642">
    <outTransitions>#1530004992551</outTransitions>
  </states>
  <states id="1530004973591" name="B" comment="">
    <plans xsi:type="alica:Plan">FailsOnOne.pml#1530069246103</plans>
    <inTransitions>#1530004992551</inTransitions>
    <outTransitions>#1530004993680</outTransitions>
  </states>
  <states id="1530004975275" name="C" comment="">
    <inTransitions>#1530004993680</inTransitions>
  </states>
  <transitions id="1530004992551" name="MISSING_NAME" comment="From A to B" msg="">
    <preCondition id="1530004993493" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1530004915641</inState>
    <outState>#1530004973591</outState>
  </transitions>
  <transitions id="1530004993680" name="MISSING_NAME" comment="AnyChildFail" msg="">
    <preCondition id="1530004994611" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1530004973591</inState>
    <outState>#1530004975275</outState>
  </transitions>
  <entryPoints id="1530004915642" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1530004915641</state>
  </entryPoints>
</alica:Plan>
