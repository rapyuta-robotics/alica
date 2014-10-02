<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1407153703092" name="PlanFive" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1407153717808" name="DefaultState" comment="" entryPoint="1407153717809">
    <plans xsi:type="alica:Plan">GoalPlan.pml#1402488870347</plans>
  </states>
  <states id="1407153985762" name="DefendState" comment="" entryPoint="1407153973706">
    <plans xsi:type="alica:Plan">Defend.pml#1402488893641</plans>
  </states>
  <states id="1407153987910" name="AttackState" comment="" entryPoint="1407153972059">
    <plans xsi:type="alica:Plan">AttackPlan.pml#1402488634525</plans>
    <plans xsi:type="alica:PlanType">Attack.pty#1407153314946</plans>
  </states>
  <states id="1407153989550" name="MidFieldState" comment="" entryPoint="1407153975075">
    <plans xsi:type="alica:Plan">MidFieldPlayPlan.pml#1402488770050</plans>
  </states>
  <entryPoints id="1407153717809" name="" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1407153717808</state>
  </entryPoints>
  <entryPoints id="1407153972059" name="AttackTask" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1407153522080</task>
    <state>#1407153987910</state>
  </entryPoints>
  <entryPoints id="1407153973706" name="DefendTask" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1402488486725</task>
    <state>#1407153985762</state>
  </entryPoints>
  <entryPoints id="1407153975075" name="MidFieldTask" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1407153536219</task>
    <state>#1407153989550</state>
  </entryPoints>
</alica:Plan>
