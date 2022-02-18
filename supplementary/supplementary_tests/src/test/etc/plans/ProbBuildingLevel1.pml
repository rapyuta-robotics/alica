{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1479557378266,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1479557378264,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1479557378265,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1479557378264,
  "masterPlan": false,
  "name": "ProbBuildingLevel1",
  "preCondition": null,
  "relativeDirectory": "ProblemModule",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "QueryPlantype.pty#1479557395790",
          "comment": "",
          "configuration": null,
          "id": 1597658636459,
          "name": "1597658636459"
        }
      ],
      "entryPoint": 1479557378266,
      "id": 1479557378265,
      "inTransitions": [],
      "name": "PTState",
      "outTransitions": [],
      "parentPlan": 1479557378264,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": [
        {
          "comment": "",
          "id": 1479557505697,
          "name": "MISSING_NAME",
          "subPlan": "QueryPlantype.pty#1479557395790",
          "subVariable": "QueryPlantype.pty#1479557463468",
          "variable": 1479557432793
        },
        {
          "comment": "",
          "id": 1479557512341,
          "name": "MISSING_NAME",
          "subPlan": "QueryPlantype.pty#1479557395790",
          "subVariable": "QueryPlantype.pty#1479557473424",
          "variable": 1479557444388
        }
      ]
    }
  ],
  "synchronisations": [],
  "transitions": [],
  "utilityThreshold": 0.1,
  "variables": [
    {
      "comment": "",
      "id": 1479557432793,
      "name": "PBL1X",
      "variableType": ""
    },
    {
      "comment": "",
      "id": 1479557444388,
      "name": "PBL1Y",
      "variableType": ""
    }
  ]
}