<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1407153663917" name="PlanThree" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1407153675524" name="DefaultState" comment="" entryPoint="1407153675525">
    <plans xsi:type="alica:Plan">PlanFour.pml#1407153683051</plans>
    <plans xsi:type="alica:Plan">GoalPlan.pml#1402488870347</plans>
  </states>
  <states id="1407153914126" name="MidFieldState" comment="" entryPoint="1407153896585">
    <plans xsi:type="alica:Plan">PlanFour.pml#1407153683051</plans>
    <plans xsi:type="alica:Plan">MidFieldPlayPlan.pml#1402488770050</plans>
  </states>
  <states id="1407153916646" name="DefendState" comment="" entryPoint="1407153899241">
    <plans xsi:type="alica:Plan">PlanFour.pml#1407153683051</plans>
    <plans xsi:type="alica:Plan">Defend.pml#1402488893641</plans>
  </states>
  <entryPoints id="1407153675525" name="" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1407153675524</state>
  </entryPoints>
  <entryPoints id="1407153896585" name="MidFieldTask" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1407153536219</task>
    <state>#1407153914126</state>
  </entryPoints>
  <entryPoints id="1407153899241" name="DefendTask" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1402488486725</task>
    <state>#1407153916646</state>
  </entryPoints>
</alica:Plan>
