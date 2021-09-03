{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1530069246105,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1530069246103,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1530069246104,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1530069246103,
  "masterPlan": false,
  "name": "FailsOnOne",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": {
    "comment": "Is not set 1",
    "conditionString": "",
    "enabled": false,
    "id": 1530069251117,
    "name": "NewRuntimeCondition",
    "pluginName": "DefaultPlugin",
    "quantifiers": [],
    "variables": []
  },
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1530069246105,
      "id": 1530069246104,
      "inTransitions": [],
      "name": "NewState",
      "outTransitions": [],
      "parentPlan": 1530069246103,
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