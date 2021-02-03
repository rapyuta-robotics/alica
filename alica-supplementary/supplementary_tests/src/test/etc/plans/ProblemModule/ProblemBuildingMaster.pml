{
  "id" : 1479556022226,
  "name" : "ProblemBuildingMaster",
  "comment" : "",
  "relativeDirectory" : "ProblemModule",
  "variables" : [ {
    "id" : 1479557337956,
    "name" : "PBMX",
    "comment" : "",
    "variableType" : ""
  }, {
    "id" : 1479557345903,
    "name" : "PBMY",
    "comment" : "",
    "variableType" : ""
  } ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1479556022228,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1479556022227,
    "plan" : 1479556022226
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1479556022227,
    "name" : "State1",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1479556022226,
    "confAbstractPlanWrappers" : [ {
      "id" : 1597658636476,
      "name" : "1597658636476",
      "comment" : "",
      "abstractPlan" : "ProblemModule/ProbBuildingLevel1.pml#1479557378264",
      "configuration" : null
    } ],
    "variableBindings" : [ {
      "id" : 1479557551007,
      "name" : "MISSING_NAME",
      "comment" : "",
      "variable" : 1479557345903,
      "subPlan" : "ProblemModule/ProbBuildingLevel1.pml#1479557378264",
      "subVariable" : "ProblemModule/ProbBuildingLevel1.pml#1479557444388"
    }, {
      "id" : 1479557555606,
      "name" : "MISSING_NAME",
      "comment" : "",
      "variable" : 1479557337956,
      "subPlan" : "ProblemModule/ProbBuildingLevel1.pml#1479557378264",
      "subVariable" : "ProblemModule/ProbBuildingLevel1.pml#1479557432793"
    } ],
    "outTransitions" : [ 1479557591331 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1479557585252,
    "name" : "State2",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1479556022226,
    "confAbstractPlanWrappers" : [ {
      "id" : 1597658636487,
      "name" : "1597658636487",
      "comment" : "",
      "abstractPlan" : "ProblemModule/ProbBuildingLevel1_1.pml#1479557664989",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1479557591331 ]
  } ],
  "transitions" : [ {
    "id" : 1479557591331,
    "name" : "MISSING_NAME",
    "comment" : "",
    "inState" : 1479556022227,
    "outState" : 1479557585252,
    "preCondition" : {
      "id" : 1479557592662,
      "name" : "MISSING_NAME",
      "comment" : "",
      "enabled" : true,
      "conditionString" : "",
      "pluginName" : "DefaultPlugin",
      "variables" : [ 1479557337956, 1479557345903 ],
      "quantifiers" : [ {
        "id" : 1479557619214,
        "name" : "MISSING_NAME",
        "comment" : "",
        "quantifierType" : "all",
        "scope" : 1479556022227,
        "sorts" : [ "X", "Y" ]
      } ]
    },
    "synchronisation" : null
  } ],
  "synchronisations" : [ ]
}
