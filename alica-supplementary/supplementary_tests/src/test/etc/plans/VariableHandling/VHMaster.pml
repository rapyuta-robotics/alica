<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1524452721452" name="VHMaster" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/VariableHandling" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1524463006078" name="NewRuntimeCondition" comment="Unrelated Condition" conditionString="VHMaster Runtime Condition" pluginName="DefaultPlugin">
    <vars>#1524463022262</vars>
    <vars>#1524463028066</vars>
  </conditions>
  <vars id="1524463022262" name="MA" comment="" Type=""/>
  <vars id="1524463028066" name="MB" comment="" Type=""/>
  <states id="1524452721453" name="NewState" comment="" entryPoint="1524452721454">
    <parametrisation id="1524463056023" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:Plan">Lvl1.pml#1524452759599</subplan>
      <subvar>Lvl1.pml#1524453336548</subvar>
      <var>#1524463022262</var>
    </parametrisation>
    <plans xsi:type="alica:Plan">Lvl1.pml#1524452759599</plans>
  </states>
  <entryPoints id="1524452721454" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1524452721453</state>
  </entryPoints>
</alica:Plan>
