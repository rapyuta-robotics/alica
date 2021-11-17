{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1524452793380,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1524452793378,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1524452793379,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    },
    {
      "comment": "",
      "id": 1524453238753,
      "maxCardinality": 1,
      "minCardinality": 1,
      "name": "AttackTask",
      "plan": 1524452793378,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 1524453248579,
      "successRequired": false,
      "task": "taskrepository.tsk#1407153522080"
    }
  ],
  "frequency": 0,
  "id": 1524452793378,
  "masterPlan": false,
  "name": "Lvl2",
  "preCondition": null,
  "relativeDirectory": "VariableHandling",
  "runtimeCondition": {
    "comment": "",
    "conditionString": "Lvl2 Runtime Condition",
    "enabled": false,
    "id": 1524453266123,
    "name": "NewRuntimeCondition",
    "pluginName": "DefaultPlugin",
    "quantifiers": [
      {
        "comment": "",
        "id": 1524453283559,
        "name": "MISSING_NAME",
        "quantifierType": "all",
        "scope": 1524453248579,
        "sorts": [
          "X",
          "Y"
        ]
      }
    ],
    "variables": [
      1524453150187,
      1524453155043,
      1524453162883
    ]
  },
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "Lvl3.pml#1524452836022",
          "comment": "",
          "configuration": null,
          "id": 1597658636528,
          "name": "1597658636528"
        }
      ],
      "entryPoint": 1524452793380,
      "id": 1524452793379,
      "inTransitions": [],
      "name": "NewState",
      "outTransitions": [],
      "parentPlan": 1524452793378,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": [
        {
          "comment": "",
          "id": 1524453170258,
          "name": "MISSING_NAME",
          "subPlan": "Lvl3.pml#1524452836022",
          "subVariable": "Lvl3.pml#1524453060294",
          "variable": 1524453155043
        },
        {
          "comment": "",
          "id": 1524453176349,
          "name": "MISSING_NAME",
          "subPlan": "Lvl3.pml#1524452836022",
          "subVariable": "Lvl3.pml#1524453054226",
          "variable": 1524453150187
        }
      ]
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1524453238753,
      "id": 1524453248579,
      "inTransitions": [],
      "name": "Dummy",
      "outTransitions": [],
      "parentPlan": 1524452793378,
      "positionWeb": {
        "x": 428,
        "y": 400
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
      "id": 1524453150187,
      "name": "L2A",
      "variableType": ""
    },
    {
      "comment": "",
      "id": 1524453155043,
      "name": "L2B",
      "variableType": ""
    },
    {
      "comment": "",
      "id": 1524453162883,
      "name": "L2C",
      "variableType": ""
    }
  ]
}