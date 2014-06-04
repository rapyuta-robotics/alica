<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1244794606469" name="WM09" comment="Paul ist der Boss!" masterPlan="true" utilityFunction="" utilityThreshold="1.0" priority="0.0" minCardinality="1" maxCardinality="2147483647">
  <states id="1244794607095" name="OwnPenalty" comment="">
    <plans xsi:type="alica:Plan">../../StandardSituations/Penalty/GO09Penalty.pml#1239552465962</plans>
    <inTransitions>#1244794606516</inTransitions>
    <outTransitions>#1244794607148</outTransitions>
  </states>
  <states id="1244794607067" name="Wander" comment="">
    <plans xsi:type="alica:PlanType">../../Util/Wander/WanderType.pty#1243263753011</plans>
    <inTransitions>#1244794606913</inTransitions>
    <inTransitions>#1244794606588</inTransitions>
    <outTransitions>#1244794607081</outTransitions>
    <outTransitions>#1244794607082</outTransitions>
    <outTransitions>#1244794607084</outTransitions>
    <outTransitions>#1244794607085</outTransitions>
    <outTransitions>#1244794607086</outTransitions>
    <outTransitions>#1244794607087</outTransitions>
    <outTransitions>#1244794607093</outTransitions>
  </states>
  <states id="1244794607031" name="OppKickOff" comment="">
    <plans xsi:type="alica:Plan">../../StandardSituations/Opp/GoalKick/WM09KOOpp.pml#1245694254359</plans>
    <inTransitions>#1244794606514</inTransitions>
    <outTransitions>#1244794607064</outTransitions>
    <outTransitions>#1244794607066</outTransitions>
  </states>
  <states id="1244794606992" name="OppThrowIn" comment="">
    <plans xsi:type="alica:Plan">../../StandardSituations/Opp/ThrowIn/WM09TIOpp.pml#1245699243172</plans>
    <inTransitions>#1244794606513</inTransitions>
    <outTransitions>#1244794607027</outTransitions>
    <outTransitions>#1244794607028</outTransitions>
  </states>
  <states id="1244794606954" name="OppGoalKick" comment="">
    <plans xsi:type="alica:Plan">../../StandardSituations/Opp/GoalKick/WM09GKOpp.pml#1245698685141</plans>
    <inTransitions>#1244794606512</inTransitions>
    <outTransitions>#1244794606989</outTransitions>
    <outTransitions>#1244794606990</outTransitions>
  </states>
  <states id="1244794606914" name="OppCorner" comment="">
    <plans xsi:type="alica:PlanType">../../StandardSituations/Opp/CornerKick/WM09CornerOppType.pty#1245404496238</plans>
    <inTransitions>#1244794606511</inTransitions>
    <outTransitions>#1244794606950</outTransitions>
    <outTransitions>#1244794606952</outTransitions>
  </states>
  <states id="1244794606872" name="GamePlay" comment="">
    <plans xsi:type="alica:Plan">../GO09/GO09Game.pml#1239132085726</plans>
    <inTransitions>#1244794606510</inTransitions>
    <inTransitions>#1244794606870</inTransitions>
    <inTransitions>#1244794606952</inTransitions>
    <inTransitions>#1244794606990</inTransitions>
    <inTransitions>#1244794607028</inTransitions>
    <inTransitions>#1244794607066</inTransitions>
    <inTransitions>#1244794606653</inTransitions>
    <inTransitions>#1244794606762</inTransitions>
    <inTransitions>#1244794606590</inTransitions>
    <inTransitions>#1244794606699</inTransitions>
    <inTransitions>#1244794606826</inTransitions>
    <inTransitions>#1244794606559</inTransitions>
    <inTransitions>#1244794607093</inTransitions>
    <inTransitions>#1335649698234</inTransitions>
    <outTransitions>#1244794606912</outTransitions>
    <outTransitions>#1244794606913</outTransitions>
  </states>
  <states id="1244794606834" name="OppFreeKick" comment="">
    <plans xsi:type="alica:Plan">../../StandardSituations/Opp/FreeKick/WM09FKOpp.pml#1245696781007</plans>
    <inTransitions>#1244794606509</inTransitions>
    <outTransitions>#1244794606868</outTransitions>
    <outTransitions>#1244794606870</outTransitions>
    <outTransitions>#1335649637550</outTransitions>
  </states>
  <states id="1244794606827" name="Joystick" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../Behaviours/Joystick.beh#1220262752817</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../Behaviours/DribbleControl.beh#1301246337391</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../Behaviours/DribbleKick.beh#1397042684562</plans>
    <inTransitions>#1244794606507</inTransitions>
    <outTransitions>#1244794606832</outTransitions>
  </states>
  <states id="1244794606763" name="GoalKick" comment="">
    <plans xsi:type="alica:Plan">../../StandardSituations/Own/GoalKick/WM09GK.pml#1245589449121</plans>
    <inTransitions>#1244794606506</inTransitions>
    <inTransitions>#1244794607084</inTransitions>
    <outTransitions>#1244794606825</outTransitions>
    <outTransitions>#1244794606826</outTransitions>
  </states>
  <states id="1244794606700" name="FreeKick" comment="">
    <plans xsi:type="alica:Plan">../../StandardSituations/Own/FreeKick/WM09FK.pml#1245014930840</plans>
    <inTransitions>#1244794606505</inTransitions>
    <inTransitions>#1244794607086</inTransitions>
    <outTransitions>#1244794606760</outTransitions>
    <outTransitions>#1244794606762</outTransitions>
  </states>
  <states id="1244794606654" name="Corner" comment="">
    <plans xsi:type="alica:PlanType">../../StandardSituations/Own/CornerKick/CornerKickType.pty#1308645378036</plans>
    <inTransitions>#1244794606504</inTransitions>
    <inTransitions>#1244794607085</inTransitions>
    <outTransitions>#1244794606696</outTransitions>
    <outTransitions>#1244794606699</outTransitions>
  </states>
  <states id="1244794606591" name="ThrowIn" comment="">
    <plans xsi:type="alica:Plan">../../StandardSituations/Own/ThrowIn/TestLSThrowin.pml#1244538879060</plans>
    <inTransitions>#1244794606500</inTransitions>
    <inTransitions>#1244794607082</inTransitions>
    <outTransitions>#1244794606652</outTransitions>
    <outTransitions>#1244794606653</outTransitions>
  </states>
  <states id="1244794606563" name="DropBall" comment="">
    <plans xsi:type="alica:PlanType">../../StandardSituations/DropBall/DrobBallType.pty#1335637483388</plans>
    <inTransitions>#1244794606503</inTransitions>
    <inTransitions>#1244794607087</inTransitions>
    <outTransitions>#1244794606587</outTransitions>
    <outTransitions>#1244794606588</outTransitions>
    <outTransitions>#1244794606590</outTransitions>
  </states>
  <states id="1244794606517" name="KickOff" comment="">
    <plans xsi:type="alica:Plan">../../StandardSituations/Own/Kickoff/WM09KO.pml#1245597431733</plans>
    <inTransitions>#1244794606502</inTransitions>
    <outTransitions>#1244794606558</outTransitions>
    <outTransitions>#1244794606559</outTransitions>
  </states>
  <states id="1244794606477" name="Stop" comment="" entryPoint="1244794606476">
    <plans xsi:type="alica:BehaviourConfiguration">../../Behaviours/SimpleLog.beh#1300107916620</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../Behaviours/StopRobot.beh#1220262752813</plans>
    <inTransitions>#1244794606587</inTransitions>
    <inTransitions>#1244794606558</inTransitions>
    <inTransitions>#1244794606652</inTransitions>
    <inTransitions>#1244794606696</inTransitions>
    <inTransitions>#1244794606760</inTransitions>
    <inTransitions>#1244794606825</inTransitions>
    <inTransitions>#1244794606832</inTransitions>
    <inTransitions>#1244794606868</inTransitions>
    <inTransitions>#1244794606912</inTransitions>
    <inTransitions>#1244794606950</inTransitions>
    <inTransitions>#1244794606989</inTransitions>
    <inTransitions>#1244794607027</inTransitions>
    <inTransitions>#1244794607064</inTransitions>
    <inTransitions>#1244794607081</inTransitions>
    <inTransitions>#1244794607148</inTransitions>
    <inTransitions>#1335196006046</inTransitions>
    <inTransitions>#1335649639452</inTransitions>
    <inTransitions>#1371020238660</inTransitions>
    <outTransitions>#1244794606500</outTransitions>
    <outTransitions>#1244794606502</outTransitions>
    <outTransitions>#1244794606503</outTransitions>
    <outTransitions>#1244794606504</outTransitions>
    <outTransitions>#1244794606505</outTransitions>
    <outTransitions>#1244794606506</outTransitions>
    <outTransitions>#1244794606507</outTransitions>
    <outTransitions>#1244794606509</outTransitions>
    <outTransitions>#1244794606510</outTransitions>
    <outTransitions>#1244794606511</outTransitions>
    <outTransitions>#1244794606512</outTransitions>
    <outTransitions>#1244794606513</outTransitions>
    <outTransitions>#1244794606514</outTransitions>
    <outTransitions>#1244794606516</outTransitions>
    <outTransitions>#1335196003879</outTransitions>
    <outTransitions>#1371020236395</outTransitions>
  </states>
  <states id="1335195956330" name="Parking" comment="">
    <plans xsi:type="alica:Plan">../../StandardSituations/Parking.pml#1335194420978</plans>
    <inTransitions>#1335196003879</inTransitions>
    <outTransitions>#1335196006046</outTransitions>
  </states>
  <states id="1335649621934" name="GoalieFKOpp" comment="">
    <plans xsi:type="alica:Plan">../../Goalie/GoalieDefendGoal.pml#1328998196845</plans>
    <inTransitions>#1335649637550</inTransitions>
    <outTransitions>#1335649639452</outTransitions>
    <outTransitions>#1335649698234</outTransitions>
  </states>
  <states id="1371020224526" name="OppPenalty" comment="">
    <plans xsi:type="alica:Plan">../../StandardSituations/Penalty/OppPenalty.pml#1371020727859</plans>
    <inTransitions>#1371020236395</inTransitions>
    <outTransitions>#1371020238660</outTransitions>
  </states>
  <transitions id="1244794607093" name="" comment="" msg="">
    <preCondition id="1244794607264" name="" comment="BallFound &amp;&amp; (Situation == Start || situation == undefined)" conditionString="" enabled="true"/>
    <inState>#1244794607067</inState>
    <outState>#1244794606872</outState>
  </transitions>
  <transitions id="1244794607087" name="" comment="BallFound &amp;&amp; situation == Dropball" msg="">
    <preCondition id="1244794607254" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794607067</inState>
    <outState>#1244794606563</outState>
  </transitions>
  <transitions id="1244794607086" name="" comment="BallFound &amp;&amp; situation == FreeKick" msg="">
    <preCondition id="1244794607252" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794607067</inState>
    <outState>#1244794606700</outState>
  </transitions>
  <transitions id="1244794607085" name="" comment="BallFound &amp;&amp; situation == Corner" msg="">
    <preCondition id="1244794607250" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794607067</inState>
    <outState>#1244794606654</outState>
  </transitions>
  <transitions id="1244794607084" name="" comment="" msg="">
    <preCondition id="1244794607248" name="" comment="BallFound &amp;&amp; situation == GoalKick" conditionString="" enabled="true"/>
    <inState>#1244794607067</inState>
    <outState>#1244794606763</outState>
  </transitions>
  <transitions id="1244794607082" name="" comment="BallFound &amp;&amp; Situation == ThrowIn" msg="">
    <preCondition id="1244794607246" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794607067</inState>
    <outState>#1244794606591</outState>
  </transitions>
  <transitions id="1244794607148" name="" comment="OwnPenalty->Stop: Situation == Stop" msg="">
    <preCondition id="1244794607244" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794607095</inState>
    <outState>#1244794606477</outState>
  </transitions>
  <transitions id="1244794606516" name="" comment="Stop->OwnPenalty: Situation == OwnPenalty" msg="">
    <preCondition id="1244794607242" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606477</inState>
    <outState>#1244794607095</outState>
  </transitions>
  <transitions id="1244794606559" name="" comment="WM09KO success or fail" msg="">
    <preCondition id="1244794607231" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606517</inState>
    <outState>#1244794606872</outState>
  </transitions>
  <transitions id="1244794606826" name="" comment="" msg="">
    <preCondition id="1244794607229" name="" comment="GO09GK success or fail" conditionString="" enabled="true"/>
    <inState>#1244794606763</inState>
    <outState>#1244794606872</outState>
  </transitions>
  <transitions id="1244794606699" name="" comment="GO09Corner (long) success || fail" msg="">
    <preCondition id="1244794607227" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606654</inState>
    <outState>#1244794606872</outState>
  </transitions>
  <transitions id="1244794606590" name="" comment="Child.Success" msg="">
    <preCondition id="1244794607225" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606563</inState>
    <outState>#1244794606872</outState>
  </transitions>
  <transitions id="1244794606762" name="" comment="" msg="">
    <preCondition id="1244794607223" name="" comment="Any child success or fail" conditionString="" enabled="true"/>
    <inState>#1244794606700</inState>
    <outState>#1244794606872</outState>
  </transitions>
  <transitions id="1244794607081" name="" comment="Situation = stop" msg="">
    <preCondition id="1244794607221" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794607067</inState>
    <outState>#1244794606477</outState>
  </transitions>
  <transitions id="1244794606588" name="" comment="BallLost" msg="">
    <preCondition id="1244794607219" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606563</inState>
    <outState>#1244794607067</outState>
  </transitions>
  <transitions id="1244794606913" name="" comment="" msg="">
    <preCondition id="1244794607215" name="" comment="BallLost" conditionString="" enabled="true"/>
    <inState>#1244794606872</inState>
    <outState>#1244794607067</outState>
  </transitions>
  <transitions id="1244794606653" name="" comment="" msg="">
    <preCondition id="1244794607213" name="" comment="TestLSThrowin success or fail" conditionString="" enabled="true"/>
    <inState>#1244794606591</inState>
    <outState>#1244794606872</outState>
  </transitions>
  <transitions id="1244794607066" name="" comment="GO09KickOffOpp success or fail" msg="">
    <preCondition id="1244794607211" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794607031</inState>
    <outState>#1244794606872</outState>
  </transitions>
  <transitions id="1244794607064" name="" comment="" msg="">
    <preCondition id="1244794607209" name="" comment="situation == stop" conditionString="" enabled="true"/>
    <inState>#1244794607031</inState>
    <outState>#1244794606477</outState>
  </transitions>
  <transitions id="1244794606514" name="" comment="situation == oppKickOff" msg="">
    <preCondition id="1244794607207" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606477</inState>
    <outState>#1244794607031</outState>
  </transitions>
  <transitions id="1244794607028" name="" comment="OppThrowIn success or fail" msg="">
    <preCondition id="1244794607205" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606992</inState>
    <outState>#1244794606872</outState>
  </transitions>
  <transitions id="1244794607027" name="" comment="" msg="">
    <preCondition id="1244794607203" name="" comment="situation == stop" conditionString="" enabled="true"/>
    <inState>#1244794606992</inState>
    <outState>#1244794606477</outState>
  </transitions>
  <transitions id="1244794606513" name="" comment="situation == OppThrowIn" msg="">
    <preCondition id="1244794607201" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606477</inState>
    <outState>#1244794606992</outState>
  </transitions>
  <transitions id="1244794606990" name="" comment="GKOpp success or fail" msg="">
    <preCondition id="1244794607199" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606954</inState>
    <outState>#1244794606872</outState>
  </transitions>
  <transitions id="1244794606989" name="" comment="situation == stop" msg="">
    <preCondition id="1244794607197" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606954</inState>
    <outState>#1244794606477</outState>
  </transitions>
  <transitions id="1244794606512" name="" comment="situation == oppgoalkick" msg="">
    <preCondition id="1244794607195" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606477</inState>
    <outState>#1244794606954</outState>
  </transitions>
  <transitions id="1244794606952" name="" comment="WM09CornerOpp plans ==  success" msg="">
    <preCondition id="1244794607193" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606914</inState>
    <outState>#1244794606872</outState>
  </transitions>
  <transitions id="1244794606950" name="" comment="" msg="">
    <preCondition id="1244794607191" name="" comment="situation == stop" conditionString="" enabled="true"/>
    <inState>#1244794606914</inState>
    <outState>#1244794606477</outState>
  </transitions>
  <transitions id="1244794606511" name="" comment="situation == opp corner" msg="">
    <preCondition id="1244794607189" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606477</inState>
    <outState>#1244794606914</outState>
  </transitions>
  <transitions id="1244794606870" name="" comment="OppFK success or fail &amp;&amp; (!I am Goalie)" msg="">
    <preCondition id="1244794607187" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606834</inState>
    <outState>#1244794606872</outState>
  </transitions>
  <transitions id="1244794606912" name="" comment="" msg="">
    <preCondition id="1244794607185" name="" comment="situation stop" conditionString="" enabled="true"/>
    <inState>#1244794606872</inState>
    <outState>#1244794606477</outState>
  </transitions>
  <transitions id="1244794606510" name="" comment="" msg="">
    <preCondition id="1244794607183" name="" comment="situation start" conditionString="" enabled="true"/>
    <inState>#1244794606477</inState>
    <outState>#1244794606872</outState>
  </transitions>
  <transitions id="1244794606868" name="" comment="situation == stop" msg="">
    <preCondition id="1244794607181" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606834</inState>
    <outState>#1244794606477</outState>
  </transitions>
  <transitions id="1244794606509" name="" comment="situation == oppFreekick" msg="">
    <preCondition id="1244794607179" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606477</inState>
    <outState>#1244794606834</outState>
  </transitions>
  <transitions id="1244794606832" name="" comment="RemoteControl stop" msg="">
    <preCondition id="1244794607177" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606827</inState>
    <outState>#1244794606477</outState>
  </transitions>
  <transitions id="1244794606507" name="" comment="RemoteControl start" msg="">
    <preCondition id="1244794607175" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606477</inState>
    <outState>#1244794606827</outState>
  </transitions>
  <transitions id="1244794606825" name="" comment="" msg="">
    <preCondition id="1244794607173" name="" comment="situation == stop" conditionString="" enabled="true"/>
    <inState>#1244794606763</inState>
    <outState>#1244794606477</outState>
  </transitions>
  <transitions id="1244794606506" name="" comment="" msg="">
    <preCondition id="1244794607171" name="" comment="situation == GoalKick" conditionString="" enabled="true"/>
    <inState>#1244794606477</inState>
    <outState>#1244794606763</outState>
  </transitions>
  <transitions id="1244794606760" name="" comment="situation == stop" msg="">
    <preCondition id="1244794607169" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606700</inState>
    <outState>#1244794606477</outState>
  </transitions>
  <transitions id="1244794606505" name="" comment="situation == freekick" msg="">
    <preCondition id="1244794607167" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606477</inState>
    <outState>#1244794606700</outState>
  </transitions>
  <transitions id="1244794606696" name="" comment="" msg="">
    <preCondition id="1244794607165" name="" comment="Situation == stop" conditionString="" enabled="true"/>
    <inState>#1244794606654</inState>
    <outState>#1244794606477</outState>
  </transitions>
  <transitions id="1244794606504" name="" comment="Situation == corner" msg="">
    <preCondition id="1244794607163" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606477</inState>
    <outState>#1244794606654</outState>
  </transitions>
  <transitions id="1244794606652" name="" comment="" msg="">
    <preCondition id="1244794607161" name="" comment="Situation == Stop" conditionString="" enabled="true"/>
    <inState>#1244794606591</inState>
    <outState>#1244794606477</outState>
  </transitions>
  <transitions id="1244794606558" name="" comment="" msg="">
    <preCondition id="1244794607159" name="" comment="Situation == Stop" conditionString="" enabled="true"/>
    <inState>#1244794606517</inState>
    <outState>#1244794606477</outState>
  </transitions>
  <transitions id="1244794606587" name="" comment="Situation == Stop" msg="">
    <preCondition id="1244794607157" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606563</inState>
    <outState>#1244794606477</outState>
  </transitions>
  <transitions id="1244794606503" name="" comment="Situation == Dropball" msg="">
    <preCondition id="1244794607155" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606477</inState>
    <outState>#1244794606563</outState>
  </transitions>
  <transitions id="1244794606502" name="" comment="Situation == KickOff" msg="">
    <preCondition id="1244794607153" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606477</inState>
    <outState>#1244794606517</outState>
  </transitions>
  <transitions id="1244794606500" name="" comment="Situation == ThrowIn" msg="">
    <preCondition id="1244794607151" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606477</inState>
    <outState>#1244794606591</outState>
  </transitions>
  <transitions id="1335196003879" name="" comment="Situation==Parking" msg="">
    <preCondition id="1335196022618" name="" comment="" conditionString="" enabled="false"/>
    <inState>#1244794606477</inState>
    <outState>#1335195956330</outState>
  </transitions>
  <transitions id="1335196006046" name="" comment="Situation!=Parking" msg="">
    <preCondition id="1335196039478" name="" comment="" conditionString="" enabled="false"/>
    <inState>#1335195956330</inState>
    <outState>#1244794606477</outState>
  </transitions>
  <transitions id="1335649637550" name="" comment="Situation == Start &amp;&amp; I am Goalie" msg="">
    <preCondition id="1335649749920" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606834</inState>
    <outState>#1335649621934</outState>
  </transitions>
  <transitions id="1335649639452" name="" comment="Situation == Stop" msg="">
    <preCondition id="1335649854643" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1335649621934</inState>
    <outState>#1244794606477</outState>
  </transitions>
  <transitions id="1335649698234" name="" comment="AnyChild.Success || StateTimeout 10s" msg="">
    <preCondition id="1335649772458" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1335649621934</inState>
    <outState>#1244794606872</outState>
  </transitions>
  <transitions id="1371020236395" name="" comment="Stop->OppPenalty: Situation == OppPenalty" msg="">
    <preCondition id="1371030695861" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1244794606477</inState>
    <outState>#1371020224526</outState>
  </transitions>
  <transitions id="1371020238660" name="" comment="OppPenalty->Stop: Situation == Stop" msg="">
    <preCondition id="1371030699166" name="" comment="" conditionString="" enabled="true"/>
    <inState>#1371020224526</inState>
    <outState>#1244794606477</outState>
  </transitions>
  <entryPoints id="1244794606476" name="" comment="" successRequired="true" minCardinality="1" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1244794606477</state>
  </entryPoints>
</alica:Plan>
