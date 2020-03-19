{
  "id" : 1407152758497,
  "name" : "MasterPlanTaskAssignment",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : true,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1407152758499,
    "name" : "1407152758499",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 2,
    "maxCardinality" : 5,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1407152758498,
    "plan" : 1407152758497
  }, {
    "id" : 1407152894887,
    "name" : "1407152894887",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 1000,
    "task" : "taskrepository.tsk#1407153522080",
    "state" : 1407152951886,
    "plan" : 1407152758497
  }, {
    "id" : 1407152900425,
    "name" : "1407152900425",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1407153536219",
    "state" : 1407152969078,
    "plan" : 1407152758497
  }, {
    "id" : 1407152902493,
    "name" : "DefendTask",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1402488486725",
    "state" : 1407152962295,
    "plan" : 1407152758497
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1407152758498,
    "name" : "AttackFirst",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407152758497,
    "abstractPlans" : [ "PlanOne.pml#1407153611768", "Attack.pty#1407153314946" ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1407152951886,
    "name" : "MidField",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407152758497,
    "abstractPlans" : [ "MidFieldPlayPlan.pml#1402488770050", "PlanOne.pml#1407153611768" ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1407152962295,
    "name" : "Defend",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407152758497,
    "abstractPlans" : [ "Defend.pml#1402488893641", "PlanOne.pml#1407153611768" ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1407152969078,
    "name" : "Goal",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407152758497,
    "abstractPlans" : [ "GoalPlan.pml#1402488870347", "PlanOne.pml#1407153611768" ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}