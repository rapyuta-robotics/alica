{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1418042796753,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1418042796751,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1418042796752,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1418042796751,
  "masterPlan": false,
  "name": "PreConditionPlan",
  "preCondition": {
    "comment": "",
    "conditionString": "Test",
    "enabled": true,
    "id": 1418042929966,
    "name": "NewPreCondition",
    "pluginName": "DefaultPlugin",
    "quantifiers": [],
    "variables": []
  },
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1418042796753,
      "id": 1418042796752,
      "inTransitions": [],
      "name": "PreConditionTest",
      "outTransitions": [],
      "parentPlan": 1418042796751,
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