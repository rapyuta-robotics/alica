{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1524452721454,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1524452721452,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1524452721453,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1524452721452,
  "masterPlan": true,
  "name": "VHMaster",
  "preCondition": null,
  "relativeDirectory": "VariableHandling",
  "runtimeCondition": {
    "comment": "Unrelated Condition",
    "conditionString": "VHMaster Runtime Condition",
    "enabled": false,
    "id": 1524463006078,
    "name": "NewRuntimeCondition",
    "pluginName": "DefaultPlugin",
    "quantifiers": [],
    "variables": [
      1524463022262,
      1524463028066
    ]
  },
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "Lvl1.pml#1524452759599",
          "comment": "",
          "configuration": null,
          "id": 1597658636570,
          "name": "1597658636570"
        }
      ],
      "entryPoint": 1524452721454,
      "id": 1524452721453,
      "inTransitions": [],
      "name": "NewState",
      "outTransitions": [],
      "parentPlan": 1524452721452,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": [
        {
          "comment": "",
          "id": 1524463056023,
          "name": "MISSING_NAME",
          "subPlan": "Lvl1.pml#1524452759599",
          "subVariable": "Lvl1.pml#1524453336548",
          "variable": 1524463022262
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
      "id": 1524463022262,
      "name": "MA",
      "variableType": ""
    },
    {
      "comment": "",
      "id": 1524463028066,
      "name": "MB",
      "variableType": ""
    }
  ]
}