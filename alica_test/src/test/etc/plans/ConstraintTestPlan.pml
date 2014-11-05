<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1414068524245" name="ConstraintTestPlan" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1414068566297" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <vars>#1414068572540</vars>
    <vars>#1414068576620</vars>
  </conditions>
  <vars id="1414068572540" name="X" comment="" Type=""/>
  <vars id="1414068576620" name="Y" comment="" Type=""/>
  <states id="1414068524246" name="constraintRunner" comment="" entryPoint="1414068524247">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/ConstraintUsingBehaviour.beh#1414068618837</plans>
  </states>
  <entryPoints id="1414068524247" name="" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1414068524246</state>
  </entryPoints>
</alica:Plan>
