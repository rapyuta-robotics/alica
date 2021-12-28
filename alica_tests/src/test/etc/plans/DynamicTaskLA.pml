{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1666138843382003218,
      "isDynamic": true,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 3337489358878214836,
      "positionWeb": {
        "x": 363,
        "y": 315.4848587164331
      },
      "state": 1633421497783210879,
      "successRequired": false,
      "task": "taskrepository.tsk#1163169622598227531"
    }
  ],
  "frequency": 0,
  "id": 3337489358878214836,
  "masterPlan": false,
  "name": "DynamicTaskLA",
  "preCondition": null,
  "relativeDirectory": "",
  "requiresParameters": false,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "DynamicTaskLB.pml#4316676367342780557",
          "comment": "",
          "configuration": null,
          "id": 288570861976016139,
          "name": ""
        }
      ],
      "entryPoint": 1666138843382003218,
      "id": 1633421497783210879,
      "inTransitions": [],
      "name": "LB",
      "outTransitions": [],
      "parentPlan": 3337489358878214836,
      "positionWeb": {
        "x": 577.3623910336239,
        "y": 301.90037359900373
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