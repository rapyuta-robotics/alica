{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1417423777546,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1417423757243,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1417423777544,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1417423757243,
  "masterPlan": false,
  "name": "GSolverTestPlan",
  "preCondition": null,
  "relativeDirectory": "GSolver",
  "runtimeCondition": {
    "comment": "",
    "conditionString": "",
    "enabled": false,
    "id": 1417424512343,
    "name": "NewRuntimeCondition",
    "pluginName": "DefaultPlugin",
    "quantifiers": [],
    "variables": [
      1417444589341,
      1417444593509
    ]
  },
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/SolverTestBehaviour.beh#1417424455986",
          "comment": "",
          "configuration": null,
          "id": 1597658636404,
          "name": "1597658636404"
        }
      ],
      "entryPoint": 1417423777546,
      "id": 1417423777544,
      "inTransitions": [],
      "name": "SolverState",
      "outTransitions": [],
      "parentPlan": 1417423757243,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": [
        {
          "comment": "",
          "id": 1417444715623,
          "name": "MISSING_NAME",
          "subPlan": "behaviours/SolverTestBehaviour.beh#1417424455986",
          "subVariable": "behaviours/SolverTestBehaviour.beh#1417444710642",
          "variable": 1417444593509
        },
        {
          "comment": "",
          "id": 1417444720874,
          "name": "MISSING_NAME",
          "subPlan": "behaviours/SolverTestBehaviour.beh#1417424455986",
          "subVariable": "behaviours/SolverTestBehaviour.beh#1417444707321",
          "variable": 1417444589341
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
      "id": 1417444589341,
      "name": "X",
      "variableType": ""
    },
    {
      "comment": "",
      "id": 1417444593509,
      "name": "Y",
      "variableType": ""
    }
  ]
}