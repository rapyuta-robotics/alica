<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1524452793378" name="Lvl2" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/VariableHandling" priority="0.0" minCardinality="1" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1524453266123" name="NewRuntimeCondition" comment="" conditionString="Lvl2 Runtime Condition" pluginName="DefaultPlugin">
    <quantifiers xsi:type="alica:ForallAgents" id="1524453283559" name="MISSING_NAME" comment="" scope="1524453248579">
      <sorts>X</sorts>
      <sorts>Y</sorts>
    </quantifiers>
    <vars>#1524453150187</vars>
    <vars>#1524453155043</vars>
    <vars>#1524453162883</vars>
  </conditions>
  <vars id="1524453150187" name="L2A" comment="" Type=""/>
  <vars id="1524453155043" name="L2B" comment="" Type=""/>
  <vars id="1524453162883" name="L2C" comment="" Type=""/>
  <states id="1524452793379" name="NewState" comment="" entryPoint="1524452793380">
    <parametrisation id="1524453170258" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:Plan">Lvl3.pml#1524452836022</subplan>
      <subvar>Lvl3.pml#1524453060294</subvar>
      <var>#1524453155043</var>
    </parametrisation>
    <parametrisation id="1524453176349" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:Plan">Lvl3.pml#1524452836022</subplan>
      <subvar>Lvl3.pml#1524453054226</subvar>
      <var>#1524453150187</var>
    </parametrisation>
    <plans xsi:type="alica:Plan">Lvl3.pml#1524452836022</plans>
  </states>
  <states id="1524453248579" name="Dummy" comment="" entryPoint="1524453238753"/>
  <entryPoints id="1524452793380" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1524452793379</state>
  </entryPoints>
  <entryPoints id="1524453238753" name="AttackTask" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../Misc/taskrepository.tsk#1407153522080</task>
    <state>#1524453248579</state>
  </entryPoints>
</alica:Plan>
