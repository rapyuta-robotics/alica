{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1629896091322,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1629896091322",
      "plan": 1629895873188,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1629896094706,
      "successRequired": false,
      "task": "taskrepository.tsk#1613371619454"
    }
  ],
  "frequency": 60,
  "id": 1629895873188,
  "masterPlan": false,
  "name": "PlanBA",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/BehBAA.beh#1629895911592",
          "comment": "",
          "configuration": null,
          "id": 1629896123291,
          "name": "1629896123291"
        }
      ],
      "entryPoint": 1629896091322,
      "id": 1629896094706,
      "inTransitions": [],
      "name": "BehBAA",
      "outTransitions": [],
      "parentPlan": 1629895873188,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [],
  "utilityThreshold": 0.0,
  "variables": []
}