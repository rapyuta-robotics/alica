<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1402489318663" name="Tackle" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1402489329141" name="AttackOpp" comment="" entryPoint="1402489329142">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviour/AttackOpp.beh#1402489366699</plans>
  </states>
  <entryPoints id="1402489329142" name="" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1402489329141</state>
  </entryPoints>
</alica:Plan>
