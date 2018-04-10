<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1407153611768" name="PlanOne" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1407153636261" name="DefaultState" comment="" entryPoint="1407153636262">
    <plans xsi:type="alica:Plan">PlanTwo.pml#1407153645238</plans>
    <plans xsi:type="alica:Plan">GoalPlan.pml#1402488870347</plans>
  </states>
  <states id="1407153807487" name="AttackState" comment="" entryPoint="1407153791141">
    <plans xsi:type="alica:Plan">PlanTwo.pml#1407153645238</plans>
    <plans xsi:type="alica:Plan">AttackPlan.pml#1402488634525</plans>
    <plans xsi:type="alica:PlanType">Attack.pty#1407153314946</plans>
  </states>
  <entryPoints id="1407153636262" name="" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1407153636261</state>
  </entryPoints>
  <entryPoints id="1407153791141" name="AttackTask" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1407153522080</task>
    <state>#1407153807487</state>
  </entryPoints>
</alica:Plan>
