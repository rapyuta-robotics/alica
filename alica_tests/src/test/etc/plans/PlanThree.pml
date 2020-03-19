{
  "id" : 1407153663917,
  "name" : "PlanThree",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1407153675525,
    "name" : "1407153675525",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1407153675524,
    "plan" : 1407153663917
  }, {
    "id" : 1407153896585,
    "name" : "MidFieldTask",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1407153536219",
    "state" : 1407153914126,
    "plan" : 1407153663917
  }, {
    "id" : 1407153899241,
    "name" : "DefendTask",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1402488486725",
    "state" : 1407153916646,
    "plan" : 1407153663917
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1407153675524,
    "name" : "DefaultState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407153663917,
    "abstractPlans" : [ "PlanFour.pml#1407153683051", "GoalPlan.pml#1402488870347" ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1407153914126,
    "name" : "MidFieldState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407153663917,
    "abstractPlans" : [ "PlanFour.pml#1407153683051", "MidFieldPlayPlan.pml#1402488770050" ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1407153916646,
    "name" : "DefendState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407153663917,
    "abstractPlans" : [ "PlanFour.pml#1407153683051", "Defend.pml#1402488893641" ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}