{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 3275358505487787998,
      "key": "task_msg",
      "type": "std::string"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 3020026408665372387,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 3523338962708569737,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 4533751772634304619,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 3523338962708569737,
  "implementationName": "UntracedPlan",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica_standard_library",
  "masterPlan": true,
  "name": "CustomWorkflowTutorial",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "WaitForMsg.plh#2324679431617631864",
          "comment": "",
          "configuration": null,
          "id": 2848149231989200191,
          "keyMapping": {
            "input": [
              {
                "childKey": "topic",
                "parentKey": null,
                "value": "task"
              }
            ],
            "output": [
              {
                "childKey": "msg",
                "parentKey": "task_msg"
              }
            ]
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 158154529973628369,
      "inTransitions": [
        492844968110048610,
        1851957926410438463
      ],
      "name": "WaitForTaskMsg",
      "outTransitions": [
        942917957687993046
      ],
      "parentPlan": 3523338962708569737,
      "positionWeb": {
        "x": 686,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "CustomWorkflow.pml#1923957580593038667",
          "comment": "",
          "configuration": null,
          "id": 3976479078999314029,
          "keyMapping": {
            "input": [
              {
                "childKey": "data",
                "parentKey": "task_msg",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1732227321262777278,
      "inTransitions": [
        942917957687993046
      ],
      "name": "ExecuteTask",
      "outTransitions": [
        492844968110048610
      ],
      "parentPlan": 3523338962708569737,
      "positionWeb": {
        "x": 944,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SpawnTurtle.beh#1689864767393644654",
          "comment": "",
          "configuration": null,
          "id": 1701601505357084523,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 3020026408665372387,
      "id": 4533751772634304619,
      "inTransitions": [],
      "name": "SpawnTurtle",
      "outTransitions": [
        1851957926410438463
      ],
      "parentPlan": 3523338962708569737,
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
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 492844968110048610,
      "inState": 1732227321262777278,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 158154529973628369,
      "pointsWeb": [
        {
          "x": 844,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3031003343149149669,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 942917957687993046,
      "inState": 158154529973628369,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 1732227321262777278,
      "pointsWeb": [
        {
          "x": 844,
          "y": 29
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3252002974980035492,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 1851957926410438463,
      "inState": 4533751772634304619,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 158154529973628369,
      "pointsWeb": [
        {
          "x": 586,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3419746199760196530,
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
