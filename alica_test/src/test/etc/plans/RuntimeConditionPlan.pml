<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1418042806575" name="RuntimeConditionPlan" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1418042967134" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin"/>
  <states id="1418042806576" name="RuntimeConditionTest" comment="" entryPoint="1418042806577"/>
  <entryPoints id="1418042806577" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1418042806576</state>
  </entryPoints>
</alica:Plan>
