<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1522377375148" name="BehaviorSuccessSpamMaster" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1522377375149" name="Normal" comment="" entryPoint="1522377375150">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/SuccessSpam.beh#1522377419087</plans>
    <inTransitions>#1522377945069</inTransitions>
    <outTransitions>#1522377944058</outTransitions>
  </states>
  <states id="1522377929290" name="Dummy" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/SuccessSpam.beh#1522377419087</plans>
    <inTransitions>#1522377944058</inTransitions>
    <outTransitions>#1522377945069</outTransitions>
  </states>
  <transitions id="1522377944058" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1522377944921" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1522377375149</inState>
    <outState>#1522377929290</outState>
  </transitions>
  <transitions id="1522377945069" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1522377946607" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1522377929290</inState>
    <outState>#1522377375149</outState>
  </transitions>
  <entryPoints id="1522377375150" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1522377375149</state>
  </entryPoints>
</alica:Plan>
