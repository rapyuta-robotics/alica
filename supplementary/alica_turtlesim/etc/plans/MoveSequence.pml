{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 3884735202079892962,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 2955879459958169673,
      "positionWeb": {
        "x": 499.3395846992555,
        "y": 408.0220653196112
      },
      "state": 2574484264265135830,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 2955879459958169673,
  "implementationName": "TracedPlan",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "MoveSequence",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "GoTo.beh#2797939494274869075",
          "comment": "",
          "configuration": null,
          "id": 1607888567506775941,
          "keyMapping": {
            "input": [
              {
                "childKey": "goal_y",
                "parentKey": null,
                "value": 8.5
              },
              {
                "childKey": "goal_x",
                "parentKey": null,
                "value": 8.5
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 615420838554955967,
      "inTransitions": [
        1876900079986123693
      ],
      "name": "GotoTopLeft",
      "outTransitions": [
        1993788045617075464
      ],
      "parentPlan": 2955879459958169673,
      "positionWeb": {
        "x": 1036.1432019308124,
        "y": 204.14641995172968
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
          "id": 1296426938536494072,
          "keyMapping": {
            "input": [
              {
                "childKey": "goal_y",
                "parentKey": null,
                "value": 8.5
              },
              {
                "childKey": "goal_x",
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
      "id": 1729530455797096653,
      "inTransitions": [
        1993788045617075464
      ],
      "name": "GotoTopRight",
      "outTransitions": [
        4134462263703326086
      ],
      "parentPlan": 2955879459958169673,
      "positionWeb": {
        "x": 688.7658889782784,
        "y": 195.4432823813355
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
          "id": 1881016199946973767,
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
      "entryPoint": 3884735202079892962,
      "id": 2574484264265135830,
      "inTransitions": [
        4134462263703326086
      ],
      "name": "GotoBottomLeft",
      "outTransitions": [
        2608336588777568599
      ],
      "parentPlan": 2955879459958169673,
      "positionWeb": {
        "x": 686,
        "y": 400
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
          "id": 4074731231953144025,
          "keyMapping": {
            "input": [
              {
                "childKey": "goal_x",
                "parentKey": null,
                "value": 8.5
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
      "id": 4356581823614089988,
      "inTransitions": [
        2608336588777568599
      ],
      "name": "GotoBottomRight",
      "outTransitions": [
        1876900079986123693
      ],
      "parentPlan": 2955879459958169673,
      "positionWeb": {
        "x": 1031.0748189863236,
        "y": 405.9388576025744
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
      "id": 1876900079986123693,
      "inState": 4356581823614089988,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 615420838554955967,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2733229888296489600,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 1993788045617075464,
      "inState": 615420838554955967,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 1729530455797096653,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3028393948947256635,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 2608336588777568599,
      "inState": 2574484264265135830,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 4356581823614089988,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4212894886227161826,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 4134462263703326086,
      "inState": 1729530455797096653,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2574484264265135830,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4510957743860464219,
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
