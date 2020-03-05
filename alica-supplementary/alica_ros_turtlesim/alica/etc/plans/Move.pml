<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1542882005838" name="Move" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="1" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1543284793605" name="CircleRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <quantifiers xsi:type="alica:ForallAgents" id="1543284852310" name="MISSING_NAME" comment="" scope="1542882005838">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
  </conditions>
  <states id="1542882041936" name="Move2Center" comment="" entryPoint="1543227886876">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/GoTo.beh#1544160989370</plans>
  </states>
  <states id="1542882494678" name="AlignCircle" comment="" entryPoint="1543227889789">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/GoTo.beh#1544160989370</plans>
  </states>
  <entryPoints id="1543227886876" name="LeaderTask" comment="" successRequired="true" minCardinality="1" maxCardinality="1">
    <task>../Misc/taskrepository.tsk#1543224732214</task>
    <state>#1542882041936</state>
  </entryPoints>
  <entryPoints id="1543227889789" name="FollowerTask" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1543224747408</task>
    <state>#1542882494678</state>
  </entryPoints>
</alica:Plan>
