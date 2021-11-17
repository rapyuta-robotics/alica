{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1524452836024,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1524452836022,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1524452836023,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1524452836022,
  "masterPlan": false,
  "name": "Lvl3",
  "preCondition": null,
  "relativeDirectory": "VariableHandling",
  "runtimeCondition": {
    "comment": "",
    "conditionString": "Lvl3 Runtime Condition",
    "enabled": false,
    "id": 1524452937477,
    "name": "NewRuntimeCondition",
    "pluginName": "DefaultPlugin",
    "quantifiers": [
      {
        "comment": "",
        "id": 1524453019900,
        "name": "MISSING_NAME",
        "quantifierType": "all",
        "scope": 1524452836022,
        "sorts": [
          "X",
          "Y"
        ]
      }
    ],
    "variables": [
      1524453054226,
      1524453060294
    ]
  },
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1524452836024,
      "id": 1524452836023,
      "inTransitions": [],
      "name": "NewState",
      "outTransitions": [],
      "parentPlan": 1524452836022,
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
  "variables": [
    {
      "comment": "",
      "id": 1524453054226,
      "name": "L3A",
      "variableType": ""
    },
    {
      "comment": "",
      "id": 1524453060294,
      "name": "L3B",
      "variableType": ""
    }
  ]
}