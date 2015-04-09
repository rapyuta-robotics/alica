<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1428508768572" name="BehaviourTriggerTestPlan" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1428508768573" name="NewState" comment="" entryPoint="1428508768574">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/TriggerA.beh#1428508312886</plans>
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/TriggerB.beh#1428508331620</plans>
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/TriggerC.beh#1428508367402</plans>
  </states>
  <entryPoints id="1428508768574" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1428508768573</state>
  </entryPoints>
</alica:Plan>
