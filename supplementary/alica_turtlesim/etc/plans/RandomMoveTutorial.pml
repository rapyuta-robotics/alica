{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 268368693041806104,
      "key": "y",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 2295131000765240190,
      "key": "x",
      "type": "double"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2483437678068619035,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 2768292828182334477,
      "positionWeb": {
        "x": 387,
        "y": 414
      },
      "state": 91880666632863452,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 2768292828182334477,
  "implementationName": "TracedPlan",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica_standard_library",
  "masterPlan": true,
  "name": "RandomMoveTutorial",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SpawnTurtle.beh#1689864767393644654",
          "comment": "",
          "configuration": null,
          "id": 4460826738374715546,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 2483437678068619035,
      "id": 91880666632863452,
      "inTransitions": [],
      "name": "SpawnTurtle",
      "outTransitions": [
        2520262450064953886
      ],
      "parentPlan": 2768292828182334477,
      "positionWeb": {
        "x": 589,
        "y": 408
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "GenerateRandom.beh#3356053372497284615",
          "comment": "",
          "configuration": null,
          "id": 1234100471240263183,
          "keyMapping": {
            "input": [
              {
                "childKey": "min",
                "parentKey": null,
                "value": 0.1
              },
              {
                "childKey": "max",
                "parentKey": null,
                "value": 10.0
              }
            ],
            "output": [
              {
                "childKey": "value",
                "parentKey": "x"
              }
            ]
          },
          "name": ""
        },
        {
          "abstractPlan": "GenerateRandom2.beh#4020999983789429596",
          "comment": "",
          "configuration": null,
          "id": 2212405775726167297,
          "keyMapping": {
            "input": [
              {
                "childKey": "min",
                "parentKey": null,
                "value": 0.1
              },
              {
                "childKey": "max",
                "parentKey": null,
                "value": 10.0
              }
            ],
            "output": [
              {
                "childKey": "value",
                "parentKey": "y"
              }
            ]
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 699418104844149540,
      "inTransitions": [
        631063377765820998,
        2520262450064953886
      ],
      "name": "ChooseNextDestination",
      "outTransitions": [
        2899315486017501859
      ],
      "parentPlan": 2768292828182334477,
      "positionWeb": {
        "x": 847,
        "y": 411
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "Idle.beh#3461702414433362536",
          "comment": "",
          "configuration": null,
          "id": 2855860879038575524,
          "keyMapping": {
            "input": [
              {
                "childKey": "time",
                "parentKey": null,
                "value": 5.0
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1353995590395471924,
      "inTransitions": [
        2886586164162798872
      ],
      "name": "Idle",
      "outTransitions": [
        631063377765820998
      ],
      "parentPlan": 2768292828182334477,
      "positionWeb": {
        "x": 1076,
        "y": 214
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "GoTo.beh#2797939494274869075",
          "comment": "",
          "configuration": null,
          "id": 2017853877374759768,
          "keyMapping": {
            "input": [
              {
                "childKey": "goal_x",
                "parentKey": "x",
                "value": null
              },
              {
                "childKey": "goal_y",
                "parentKey": "y",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1474370998699018467,
      "inTransitions": [
        2899315486017501859
      ],
      "name": "GoToDestination",
      "outTransitions": [
        2886586164162798872
      ],
      "parentPlan": 2768292828182334477,
      "positionWeb": {
        "x": 1207,
        "y": 411
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 631063377765820998,
      "inState": 1353995590395471924,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 699418104844149540,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 906579184271078477,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 2520262450064953886,
      "inState": 91880666632863452,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 699418104844149540,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2482416891748399139,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 2886586164162798872,
      "inState": 1474370998699018467,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 1353995590395471924,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2472540819854080245,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2",
      "id": 2899315486017501859,
      "inState": 699418104844149540,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 1474370998699018467,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3454146765593914364,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.0,
  "variables": []
}
