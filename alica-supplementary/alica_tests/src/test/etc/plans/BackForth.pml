<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1529456584982" name="BackForth" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1529456584983" name="First" comment="" entryPoint="1529456584984">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/CountIndefinitely.beh#1529456686038</plans>
    <inTransitions>#1529456610905</inTransitions>
    <outTransitions>#1529456609989</outTransitions>
  </states>
  <states id="1529456591410" name="Second" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/CountIndefinitely.beh#1529456686038</plans>
    <inTransitions>#1529456609989</inTransitions>
    <outTransitions>#1529456610905</outTransitions>
  </states>
  <transitions id="1529456609989" name="MISSING_NAME" comment="Forth" msg="">
    <preCondition id="1529456610697" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1529456584983</inState>
    <outState>#1529456591410</outState>
  </transitions>
  <transitions id="1529456610905" name="MISSING_NAME" comment="Back" msg="">
    <preCondition id="1529456611916" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1529456591410</inState>
    <outState>#1529456584983</outState>
  </transitions>
  <entryPoints id="1529456584984" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1529456584983</state>
  </entryPoints>
</alica:Plan>
