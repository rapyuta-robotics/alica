{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "id": 110523600591030675,
      "key": "xmin",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 3518934415848973217,
      "key": "xmax",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 181181142658587283,
      "key": "ymin",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 3833947625034180743,
      "key": "ymax",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 2295131000765240190,
      "key": "x",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 268368693041806104,
      "key": "y",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 3595251136493112330,
      "key": "idletime",
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
  "inheritBlackboard": false,
  "libraryName": "libalica-turtlesim",
  "masterPlan": true,
  "name": "RandomMoveTutorial",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/SpawnTurtle.beh#1689864767393644654",
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
          "abstractPlan": "behaviours/GenerateRandom.beh#3356053372497284615",
          "comment": "",
          "configuration": null,
          "id": 1234100471240263183,
          "keyMapping": {
            "input": [
              {
                "childKey": "min",
                "parentKey": "xmin",
                "value": null
              },
              {
                "childKey": "max",
                "parentKey": "xmax",
                "value": null
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
          "abstractPlan": "behaviours/GenerateRandom.beh#3356053372497284615",
          "comment": "",
          "configuration": null,
          "id": 1830155368048946440,
          "keyMapping": {
            "input": [
              {
                "childKey": "min",
                "parentKey": "ymin",
                "value": null
              },
              {
                "childKey": "max",
                "parentKey": "ymax",
                "value": null
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
        1813202192236879166
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
          "abstractPlan": "behaviours/Idle.beh#3461702414433362536",
          "comment": "",
          "configuration": null,
          "id": 2855860879038575524,
          "keyMapping": {
            "input": [
              {
                "childKey": "time",
                "parentKey": "idletime",
                "value": null
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
          "abstractPlan": "behaviours/GoTo.beh#2797939494274869075",
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
        1813202192236879166
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
      "condition": "conditions/ConditionRepository.cnd#1",
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
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#2",
      "id": 1813202192236879166,
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
        "id": 207004019330646766,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#1",
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
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#1",
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
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.0,
  "variables": []
}
