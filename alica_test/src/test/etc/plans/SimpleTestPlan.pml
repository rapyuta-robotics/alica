<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1412252439925" name="SimpleTestPlan" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1412252439926" name="TestState1" comment="" entryPoint="1412252439927">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/Attack.beh#1402488866727</plans>
  </states>
  <entryPoints id="1412252439927" name="" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1412252439926</state>
  </entryPoints>
</alica:Plan>
