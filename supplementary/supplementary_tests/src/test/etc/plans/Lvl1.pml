{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1524452759601,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1524452759599,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1524453481856,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1524452759599,
  "masterPlan": false,
  "name": "Lvl1",
  "preCondition": null,
  "relativeDirectory": "VariableHandling",
  "runtimeCondition": {
    "comment": "",
    "conditionString": "Lvl1 Runtime Condition",
    "enabled": false,
    "id": 1524453470580,
    "name": "NewRuntimeCondition",
    "pluginName": "DefaultPlugin",
    "quantifiers": [],
    "variables": [
      1524453326397,
      1524453331530
    ]
  },
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "VHPLanType.pty#1524452770528",
          "comment": "",
          "configuration": null,
          "id": 1597658636548,
          "name": "1597658636548"
        }
      ],
      "entryPoint": null,
      "id": 1524452759600,
      "inTransitions": [
        1524453490345
      ],
      "name": "NewState",
      "outTransitions": [],
      "parentPlan": 1524452759599,
      "positionWeb": {
        "x": 686,
        "y": 200
      },
      "type": "State",
      "variableBindings": [
        {
          "comment": "",
          "id": 1524453528169,
          "name": "MISSING_NAME",
          "subPlan": "VHPLanType.pty#1524452770528",
          "subVariable": "VHPLanType.pty#1524453441020",
          "variable": 1524453331530
        },
        {
          "comment": "",
          "id": 1524453534656,
          "name": "MISSING_NAME",
          "subPlan": "VHPLanType.pty#1524452770528",
          "subVariable": "VHPLanType.pty#1524453433443",
          "variable": 1524453326397
        }
      ]
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1524452759601,
      "id": 1524453481856,
      "inTransitions": [],
      "name": "BeforeTrans",
      "outTransitions": [
        1524453490345
      ],
      "parentPlan": 1524452759599,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "Lvl1 Transition",
      "id": 1524453490345,
      "inState": 1524453481856,
      "name": "MISSING_NAME",
      "outState": 1524452759600,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1524453491764,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [
          {
            "comment": "",
            "id": 1524453546255,
            "name": "MISSING_NAME",
            "quantifierType": "all",
            "scope": 1524452759601,
            "sorts": [
              "X",
              "Y"
            ]
          }
        ],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.1,
  "variables": [
    {
      "comment": "",
      "id": 1524453326397,
      "name": "L1A",
      "variableType": ""
    },
    {
      "comment": "",
      "id": 1524453331530,
      "name": "L1B",
      "variableType": ""
    },
    {
      "comment": "",
      "id": 1524453336548,
      "name": "L1C",
      "variableType": ""
    }
  ]
}