<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1407152758497" name="MasterPlanTaskAssignment" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" priority="0.0" minCardinality="2" maxCardinality="2147483647">
  <states id="1407152758498" name="AttackFirst" comment="" entryPoint="1407152758499">
    <plans xsi:type="alica:Plan">PlanOne.pml#1407153611768</plans>
    <plans xsi:type="alica:PlanType">Attack.pty#1407153314946</plans>
  </states>
  <states id="1407152951886" name="MidField" comment="" entryPoint="1407152894887">
    <plans xsi:type="alica:Plan">MidFieldPlayPlan.pml#1402488770050</plans>
    <plans xsi:type="alica:Plan">PlanOne.pml#1407153611768</plans>
  </states>
  <states id="1407152962295" name="Defend" comment="" entryPoint="1407152902493">
    <plans xsi:type="alica:Plan">Defend.pml#1402488893641</plans>
    <plans xsi:type="alica:Plan">PlanOne.pml#1407153611768</plans>
  </states>
  <states id="1407152969078" name="Goal" comment="" entryPoint="1407152900425">
    <plans xsi:type="alica:Plan">GoalPlan.pml#1402488870347</plans>
    <plans xsi:type="alica:Plan">PlanOne.pml#1407153611768</plans>
  </states>
  <entryPoints id="1407152758499" name="" comment="" successRequired="false" minCardinality="2" maxCardinality="5">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1407152758498</state>
  </entryPoints>
  <entryPoints id="1407152894887" name="" comment="" successRequired="false" minCardinality="0" maxCardinality="1000">
    <task>../Misc/taskrepository.tsk#1407153522080</task>
    <state>#1407152951886</state>
  </entryPoints>
  <entryPoints id="1407152900425" name="" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1407153536219</task>
    <state>#1407152969078</state>
  </entryPoints>
  <entryPoints id="1407152902493" name="DefendTask" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1402488486725</task>
    <state>#1407152962295</state>
  </entryPoints>
</alica:Plan>
