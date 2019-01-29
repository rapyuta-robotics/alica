<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1479557664989" name="ProbBuildingLevel1_1" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <vars id="1479718387346" name="PBL1_1X" comment="" Type=""/>
  <vars id="1479718398960" name="PBL1_1Y" comment="" Type=""/>
  <states id="1479557690962" name="MiddleState" comment="" entryPoint="1479557690963">
    <plans xsi:type="alica:Plan">QueryPlan2.pml#1479718449392</plans>
  </states>
  <entryPoints id="1479557690963" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1479557690962</state>
  </entryPoints>
</alica:Plan>
