{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1530004940654,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1530004940652,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1530004940653,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1530004940652,
  "masterPlan": true,
  "name": "HandleFailExplicitMaster",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "HandleFailExplicit.pml#1530004915640",
          "comment": "",
          "configuration": null,
          "id": 1587718662811,
          "name": "1587718662811"
        }
      ],
      "entryPoint": 1530004940654,
      "id": 1530004940653,
      "inTransitions": [],
      "name": "NewState",
      "outTransitions": [],
      "parentPlan": 1530004940652,
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
  "utilityThreshold": 0.1,
  "variables": []
}