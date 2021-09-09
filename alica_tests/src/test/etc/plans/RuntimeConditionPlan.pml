{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1418042806577,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1418042806575,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1418042806576,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1418042806575,
  "masterPlan": false,
  "name": "RuntimeConditionPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": {
    "comment": "",
    "conditionString": "",
    "enabled": false,
    "id": 1418042967134,
    "name": "NewRuntimeCondition",
    "pluginName": "DefaultPlugin",
    "quantifiers": [],
    "variables": []
  },
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1418042806577,
      "id": 1418042806576,
      "inTransitions": [],
      "name": "RuntimeConditionTest",
      "outTransitions": [],
      "parentPlan": 1418042806575,
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