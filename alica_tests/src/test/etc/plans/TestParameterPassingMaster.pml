{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 4058387577648167302,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "ParameterPassingMasterEP",
      "plan": 1179066429431332055,
      "positionWeb": {
        "x": 265,
        "y": 263
      },
      "state": 2069338196796962570,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1179066429431332055,
  "masterPlan": true,
  "name": "TestParameterPassingMaster",
  "preCondition": null,
  "relativeDirectory": "",
  "requiresParameters": false,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "TestParameterPassing.pml#1692837668719979457",
          "comment": "",
          "configuration": null,
          "id": 105160539449888459,
          "name": ""
        }
      ],
      "entryPoint": 4058387577648167302,
      "id": 2069338196796962570,
      "inTransitions": [],
      "name": "ParameterPassingRunSubPlan",
      "outTransitions": [],
      "parentPlan": 1179066429431332055,
      "positionWeb": {
        "x": 627,
        "y": 277
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [],
  "utilityThreshold": 0.0,
  "variables": [],
  "inheritBlackboard" : false
}