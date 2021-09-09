{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1479556074051,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1479556074049,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1479556074050,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1479556074049,
  "masterPlan": false,
  "name": "QueryPlan1",
  "preCondition": null,
  "relativeDirectory": "ProblemModule",
  "runtimeCondition": {
    "comment": "",
    "conditionString": "",
    "enabled": false,
    "id": 1479556084493,
    "name": "NewRuntimeCondition",
    "pluginName": "DefaultPlugin",
    "quantifiers": [
      {
        "comment": "",
        "id": 1479556181307,
        "name": "MISSING_NAME",
        "quantifierType": "all",
        "scope": 1479556074050,
        "sorts": [
          "X",
          "Y",
          "Z"
        ]
      }
    ],
    "variables": [
      1479556220234,
      1479556572534
    ]
  },
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/QueryBehaviour1.beh#1479556104511",
          "comment": "",
          "configuration": null,
          "id": 1597658636505,
          "name": "1597658636505"
        }
      ],
      "entryPoint": 1479556074051,
      "id": 1479556074050,
      "inTransitions": [],
      "name": "QueryState1",
      "outTransitions": [],
      "parentPlan": 1479556074049,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": [
        {
          "comment": "",
          "id": 1479557255352,
          "name": "MISSING_NAME",
          "subPlan": "behaviours/QueryBehaviour1.beh#1479556104511",
          "subVariable": "behaviours/QueryBehaviour1.beh#1479556246733",
          "variable": 1479556220234
        },
        {
          "comment": "",
          "id": 1479557270121,
          "name": "MISSING_NAME",
          "subPlan": "behaviours/QueryBehaviour1.beh#1479556104511",
          "subVariable": "behaviours/QueryBehaviour1.beh#1479557263650",
          "variable": 1479556572534
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
      "id": 1479556220234,
      "name": "QP1X",
      "variableType": ""
    },
    {
      "comment": "",
      "id": 1479556572534,
      "name": "QP1Y",
      "variableType": ""
    }
  ]
}