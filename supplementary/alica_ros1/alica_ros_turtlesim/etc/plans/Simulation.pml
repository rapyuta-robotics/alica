{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2741715629576575326,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 1,
      "name": "",
      "plan": 2425328142973735249,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 3997532517592149463,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 2425328142973735249,
  "implementationName": "TracedPlan",
  "inheritBlackboard": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "Simulation",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "Make the formation. Leave the state if a trigger to do so is received",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "WaitForTrigger.beh#1320667069122998665",
          "comment": "",
          "configuration": null,
          "id": 428513459178575844,
          "keyMapping": {
            "input": [
              {
                "childKey": "topic",
                "parentKey": null,
                "value": "leave_formation"
              }
            ],
            "output": []
          },
          "name": ""
        },
        {
          "abstractPlan": "MakeFormation.pml#1889749086610694100",
          "comment": "",
          "configuration": null,
          "id": 496472308860171093,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2405597980801916441,
      "inTransitions": [
        1019769038957717905,
        2467937620932239445
      ],
      "name": "MakeFormation",
      "outTransitions": [
        4195013669615888372
      ],
      "parentPlan": 2425328142973735249,
      "positionWeb": {
        "x": 748.684223242413,
        "y": 210.7241732946835
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "Go to the corner and wait until a trigger to join the formation is received",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "WaitForTrigger.beh#1320667069122998665",
          "comment": "",
          "configuration": null,
          "id": 569238231169711866,
          "keyMapping": {
            "input": [
              {
                "childKey": "topic",
                "parentKey": null,
                "value": "join_formation"
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3064881354318777214,
      "inTransitions": [
        3147792994605157106
      ],
      "name": "WaitOutOfFormation",
      "outTransitions": [
        1019769038957717905
      ],
      "parentPlan": 2425328142973735249,
      "positionWeb": {
        "x": 756.0311303883144,
        "y": -4.106345813178368
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "TeleportToRandomPosition.pml#847199748749956244",
          "comment": "",
          "configuration": null,
          "id": 108848092215946015,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 2741715629576575326,
      "id": 3997532517592149463,
      "inTransitions": [],
      "name": "Teleport2RandomPosition",
      "outTransitions": [
        2467937620932239445
      ],
      "parentPlan": 2425328142973735249,
      "positionWeb": {
        "x": 428,
        "y": 200
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
          "id": 3025259812511836525,
          "keyMapping": {
            "input": [
              {
                "childKey": "goal_x",
                "parentKey": null,
                "value": 1.5
              },
              {
                "childKey": "goal_y",
                "parentKey": null,
                "value": 1.5
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 4082412132485660911,
      "inTransitions": [
        4195013669615888372
      ],
      "name": "LeaveFormation",
      "outTransitions": [
        3147792994605157106
      ],
      "parentPlan": 2425328142973735249,
      "positionWeb": {
        "x": 994.4978480414026,
        "y": 122.03382606183122
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
      "id": 1019769038957717905,
      "inState": 3064881354318777214,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2405597980801916441,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3392326148903943635,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 2467937620932239445,
      "inState": 3997532517592149463,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2405597980801916441,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3077426683382044276,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 3147792994605157106,
      "inState": 4082412132485660911,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3064881354318777214,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2389656767398160456,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 4195013669615888372,
      "inState": 2405597980801916441,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 4082412132485660911,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3360288480763978554,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": -1.0,
  "variables": []
}
