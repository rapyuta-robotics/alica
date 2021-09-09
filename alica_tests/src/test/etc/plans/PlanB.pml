{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1629896055805,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1629896055805",
      "plan": 1629895853508,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1629896057548,
      "successRequired": false,
      "task": "taskrepository.tsk#1613371619454"
    }
  ],
  "frequency": 50,
  "id": 1629895853508,
  "masterPlan": false,
  "name": "PlanB",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanBA.pml#1629895873188",
          "comment": "",
          "configuration": null,
          "id": 1629896077656,
          "name": "1629896077656"
        }
      ],
      "entryPoint": 1629896055805,
      "id": 1629896057548,
      "inTransitions": [],
      "name": "PlanBA",
      "outTransitions": [],
      "parentPlan": 1629895853508,
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