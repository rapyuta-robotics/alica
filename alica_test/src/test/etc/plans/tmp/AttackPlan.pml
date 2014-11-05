<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1402488634525" name="AttackPlan" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <vars id="1403772778288" name="TestVar1" comment="This should be a double! :)" Type="double"/>
  <vars id="1403772797469" name="VarTest2" comment="Another int variable" Type="int"/>
  <vars id="1403772816953" name="NewVar" comment="Here is no type given !?" Type=""/>
  <vars id="1403772834750" name="ABC" comment="..." Type="FOL"/>
  <states id="1402488646220" name="Attack" comment="" entryPoint="1402488646221">
    <plans xsi:type="alica:Plan">Tackle.pml#1402489318663</plans>
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/AttackOpp.beh#1402489366699</plans>
    <inTransitions>#1402489460694</inTransitions>
    <outTransitions>#1402489459382</outTransitions>
  </states>
  <states id="1402489396914" name="Shoot" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/Attack.beh#1402488866727</plans>
    <inTransitions>#1402489459382</inTransitions>
    <outTransitions>#1402489460694</outTransitions>
  </states>
  <transitions id="1402489459382" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1402489460549" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true">
      <quantifiers xsi:type="alica:ForallAgents" id="1403773214317" name="MISSING_NAME" comment="" scope="1402488634525">
        <sorts>X</sorts>
        <sorts>Y</sorts>
      </quantifiers>
      <quantifiers xsi:type="alica:ForallAgents" id="1403773224776" name="MISSING_NAME" comment="" scope="1402488646220">
        <sorts>A</sorts>
        <sorts>B</sorts>
      </quantifiers>
      <quantifiers xsi:type="alica:ForallAgents" id="1403773234841" name="MISSING_NAME" comment="" scope="1402489396914">
        <sorts>another one</sorts>
      </quantifiers>
      <quantifiers xsi:type="alica:ForallAgents" id="1403773248357" name="MISSING_NAME" comment="" scope="1402488646221">
        <sorts>TaskQuantifier</sorts>
      </quantifiers>
      <parameters id="1403773042559" name="MISSING_NAME" comment="" key="parameter_formular" value="ACED000574000E6861766554776F42616C6C732020"/>
      <parameters id="1403773042561" name="MISSING_NAME" comment="" key="parameter_resolved_formular" value="ACED00057400202868617665416E6F7468657242616C6C2026206861766542616C6C2020292020"/>
      <parameters id="1403773042563" name="MISSING_NAME" comment="" key="parameter_operands" value="ACED0005737200136A6176612E7574696C2E41727261794C6973747881D21D99C7619D03000149000473697A6578700000000277040000000274000F68617665416E6F7468657242616C6C7400086861766542616C6C78"/>
      <vars>#1403772834750</vars>
      <vars>#1403772778288</vars>
    </preCondition>
    <inState>#1402488646220</inState>
    <outState>#1402489396914</outState>
  </transitions>
  <transitions id="1402489460694" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1402489462088" name="Condition-Name-Shoot-Attack" comment="" conditionString="Some nice comment!" pluginName="DefaultPlugin" enabled="true">
      <vars>#1403772778288</vars>
      <vars>#1403772797469</vars>
      <vars>#1403772816953</vars>
      <vars>#1403772834750</vars>
    </preCondition>
    <inState>#1402489396914</inState>
    <outState>#1402488646220</outState>
  </transitions>
  <entryPoints id="1402488646221" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1402488646220</state>
  </entryPoints>
</alica:Plan>
