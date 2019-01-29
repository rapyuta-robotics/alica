<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1524452836022" name="Lvl3" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/VariableHandling" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1524452937477" name="NewRuntimeCondition" comment="" conditionString="Lvl3 Runtime Condition" pluginName="DefaultPlugin">
    <quantifiers xsi:type="alica:ForallAgents" id="1524453019900" name="MISSING_NAME" comment="" scope="1524452836022">
      <sorts>X</sorts>
      <sorts>Y</sorts>
    </quantifiers>
    <vars>#1524453054226</vars>
    <vars>#1524453060294</vars>
  </conditions>
  <vars id="1524453054226" name="L3A" comment="" Type=""/>
  <vars id="1524453060294" name="L3B" comment="" Type=""/>
  <states id="1524452836023" name="NewState" comment="" entryPoint="1524452836024"/>
  <entryPoints id="1524452836024" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1524452836023</state>
  </entryPoints>
</alica:Plan>
