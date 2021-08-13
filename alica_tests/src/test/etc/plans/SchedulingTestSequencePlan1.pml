{
  "id" : 1614963946725,
  "name" : "SchedulingTestSequencePlan1",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1614963977287,
    "name" : "1614963977287",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1613372009777",
    "state" : 1614963979424,
    "plan" : 1614963946725
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1614963979424,
    "name" : "InitSequencePlan1",
    "comment" : "",
    "entryPoint" : 1614963977287,
    "parentPlan" : 1614963946725,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1614964566530 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1614964540694,
    "name" : "InitSequenceSubPlan1",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1614963946725,
    "confAbstractPlanWrappers" : [ {
      "id" : 1614964583934,
      "name" : "1614964583934",
      "comment" : "",
      "abstractPlan" : "SchedulingTestSequenceSubPlan1.pml#1614964379654",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1614964572494 ],
    "inTransitions" : [ 1614964566530 ]
  }, {
    "type" : "State",
    "id" : 1614964541828,
    "name" : "InitSequenceSubPlan2",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1614963946725,
    "confAbstractPlanWrappers" : [ {
      "id" : 1614964588445,
      "name" : "1614964588445",
      "comment" : "",
      "abstractPlan" : "SchedulingTestSequenceSubPlan2.pml#1614964444419",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1614964575552 ],
    "inTransitions" : [ 1614964572494 ]
  }, {
    "type" : "State",
    "id" : 1614964542678,
    "name" : "InitSequenceSubPlan3",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1614963946725,
    "confAbstractPlanWrappers" : [ {
      "id" : 1614964591096,
      "name" : "1614964591096",
      "comment" : "",
      "abstractPlan" : "SchedulingTestSequenceSubPlan3.pml#1614964478264",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1614964578015 ],
    "inTransitions" : [ 1614964575552 ]
  }, {
    "type" : "State",
    "id" : 1614964543300,
    "name" : "TerminateSequenceSubPlan3",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1614963946725,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1614964578015 ]
  } ],
  "transitions" : [ {
    "id" : 1614964566530,
    "name" : "FromInitSequencePlan1To InitSequenceSubPlan1",
    "comment" : "MISSING_COMMENT",
    "inState" : 1614963979424,
    "outState" : 1614964540694,
    "preCondition" : {
      "id" : 1614964566531,
      "name" : "1614964566531",
      "comment" : "",
      "enabled" : true,
      "conditionString" : null,
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1614964572494,
    "name" : "FromInitSequenceSubPlan1To InitSequenceSubPlan2",
    "comment" : "MISSING_COMMENT",
    "inState" : 1614964540694,
    "outState" : 1614964541828,
    "preCondition" : {
      "id" : 1614964572495,
      "name" : "1614964572495",
      "comment" : "",
      "enabled" : true,
      "conditionString" : null,
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1614964575552,
    "name" : "FromInitSequenceSubPlan2To InitSequenceSubPlan3",
    "comment" : "MISSING_COMMENT",
    "inState" : 1614964541828,
    "outState" : 1614964542678,
    "preCondition" : {
      "id" : 1614964575553,
      "name" : "1614964575553",
      "comment" : "",
      "enabled" : true,
      "conditionString" : null,
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1614964578015,
    "name" : "FromInitSequenceSubPlan3To TerminateSequenceSubPlan3",
    "comment" : "MISSING_COMMENT",
    "inState" : 1614964542678,
    "outState" : 1614964543300,
    "preCondition" : {
      "id" : 1614964578016,
      "name" : "1614964578016",
      "comment" : "",
      "enabled" : true,
      "conditionString" : null,
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  } ],
  "synchronisations" : [ ]
}