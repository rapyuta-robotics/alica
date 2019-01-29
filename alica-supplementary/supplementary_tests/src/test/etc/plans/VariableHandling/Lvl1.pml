<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1524452759599" name="Lvl1" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/VariableHandling" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1524453470580" name="NewRuntimeCondition" comment="" conditionString="Lvl1 Runtime Condition" pluginName="DefaultPlugin">
    <vars>#1524453326397</vars>
    <vars>#1524453331530</vars>
  </conditions>
  <vars id="1524453326397" name="L1A" comment="" Type=""/>
  <vars id="1524453331530" name="L1B" comment="" Type=""/>
  <vars id="1524453336548" name="L1C" comment="" Type=""/>
  <states id="1524452759600" name="NewState" comment="">
    <parametrisation id="1524453528169" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:PlanType">VHPlanType.pty#1524452770528</subplan>
      <subvar>VHPlanType.pty#1524453441020</subvar>
      <var>#1524453331530</var>
    </parametrisation>
    <parametrisation id="1524453534656" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:PlanType">VHPlanType.pty#1524452770528</subplan>
      <subvar>VHPlanType.pty#1524453433443</subvar>
      <var>#1524453326397</var>
    </parametrisation>
    <plans xsi:type="alica:PlanType">VHPlanType.pty#1524452770528</plans>
    <inTransitions>#1524453490345</inTransitions>
  </states>
  <states id="1524453481856" name="BeforeTrans" comment="" entryPoint="1524452759601">
    <outTransitions>#1524453490345</outTransitions>
  </states>
  <transitions id="1524453490345" name="MISSING_NAME" comment="Lvl1 Transition" msg="">
    <preCondition id="1524453491764" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true">
      <quantifiers xsi:type="alica:ForallAgents" id="1524453546255" name="MISSING_NAME" comment="" scope="1524452759601">
        <sorts>X</sorts>
        <sorts>Y</sorts>
      </quantifiers>
    </preCondition>
    <inState>#1524453481856</inState>
    <outState>#1524452759600</outState>
  </transitions>
  <entryPoints id="1524452759601" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1524453481856</state>
  </entryPoints>
</alica:Plan>
