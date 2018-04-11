<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1414403413451" name="AuthorityTest" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.0" priority="0.0" minCardinality="2" maxCardinality="2">
  <states id="1414403429950" name="UpperState" comment="" entryPoint="1414403429951"/>
  <states id="1414403553717" name="LowerState" comment="" entryPoint="1414403522424"/>
  <entryPoints id="1414403429951" name="" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1414403429950</state>
  </entryPoints>
  <entryPoints id="1414403522424" name="AttackTask" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../Misc/taskrepository.tsk#1407153522080</task>
    <state>#1414403553717</state>
  </entryPoints>
</alica:Plan>
