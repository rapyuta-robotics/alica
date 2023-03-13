{
  "blackboard": [
    {
      "access": "protected",
      "comment": "Name of the custom workflow to be executed",
      "id": 4266549331742849071,
      "key": "workflow",
      "type": "std::string"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 3749207483126254197,
      "key": "blackboardBlueprint",
      "type": "std::any"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 1051359321839093785,
      "key": "task",
      "type": "std::string"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 2627617029392114707,
      "key": "blackboardKeys",
      "type": "std::any"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 355127835588429254,
      "key": "blackboard",
      "type": "std::any"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 925753448706367246,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 4189894247945850072,
      "positionWeb": {
        "x": 146.83799601091755,
        "y": 450.5056818181818
      },
      "state": 1916846391743147637,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 4189894247945850072,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "libalica-turtlesim",
  "masterPlan": true,
  "name": "CustomWorkflowsTutorial",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "Move the robot to an adhoc location",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "AdhocMoveWorkflow.pml#3683805017237692753",
          "comment": "",
          "configuration": null,
          "id": 4017118265736324526,
          "keyMapping": {
            "input": [
              {
                "childKey": "task",
                "parentKey": "task",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1127883625959574268,
      "inTransitions": [
        421626603653342657
      ],
      "name": "AdhocMove",
      "outTransitions": [
        2232504332093969980
      ],
      "parentPlan": 4189894247945850072,
      "positionWeb": {
        "x": 1421.3041692108766,
        "y": 712.6401889299954
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "WaitForMsg.beh#2324679431617631864",
          "comment": "",
          "configuration": null,
          "id": 2037530943404311588,
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
                "parentKey": "task"
              }
            ]
          },
          "name": ""
        }
      ],
      "entryPoint": 925753448706367246,
      "id": 1916846391743147637,
      "inTransitions": [
        1636776243852871853,
        2232504332093969980
      ],
      "name": "WaitForTaskMsg",
      "outTransitions": [
        2139084451508981149
      ],
      "parentPlan": 4189894247945850072,
      "positionWeb": {
        "x": 430.29462786059213,
        "y": 437.48409615788364
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PopulateBlackboardFromJson.beh#3218372711073374685",
          "comment": "",
          "configuration": null,
          "id": 178599925604300957,
          "keyMapping": {
            "input": [
              {
                "childKey": "blackboard",
                "parentKey": "blackboard",
                "value": null
              },
              {
                "childKey": "blackboardBlueprint",
                "parentKey": "blackboardBlueprint",
                "value": null
              },
              {
                "childKey": "blackboardKeys",
                "parentKey": "blackboardKeys",
                "value": null
              },
              {
                "childKey": "json",
                "parentKey": "task",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2634651831492456620,
      "inTransitions": [
        2139084451508981149
      ],
      "name": "PopulateFromTaskMsg",
      "outTransitions": [
        3277683513653767607
      ],
      "parentPlan": 4189894247945850072,
      "positionWeb": {
        "x": 942.2880879861085,
        "y": 449.7111849907669
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "Move item(s) from location A to location B",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "TransportWorkflow.pml#674771508287965487",
          "comment": "",
          "configuration": null,
          "id": 4023168320102335430,
          "keyMapping": {
            "input": [
              {
                "childKey": "task",
                "parentKey": "task",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2803307801097600741,
      "inTransitions": [
        598563885758171039
      ],
      "name": "Transport",
      "outTransitions": [
        1636776243852871853
      ],
      "parentPlan": 4189894247945850072,
      "positionWeb": {
        "x": 1394.6149967673575,
        "y": 156.05126432240223
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 662315042812135154,
      "inTransitions": [
        3277683513653767607
      ],
      "name": "Decider",
      "outTransitions": [
        598563885758171039,
        421626603653342657
      ],
      "parentPlan": 4189894247945850072,
      "positionWeb": {
        "x": 1298.2631408309248,
        "y": 446.3924694502714
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#976473172990372064",
      "id": 598563885758171039,
      "inState": 662315042812135154,
      "keyMapping": {
        "input": [
          {
            "childKey": "lhs",
            "parentKey": "workflow",
            "value": null
          },
          {
            "childKey": "rhs",
            "parentKey": null,
            "value": "transport"
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2803307801097600741,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4372606493269818631,
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
      "id": 3277683513653767607,
      "inState": 2634651831492456620,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 662315042812135154,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 314965315740192311,
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
      "id": 1636776243852871853,
      "inState": 2803307801097600741,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 1916846391743147637,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4503769996376629474,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#976473172990372064",
      "id": 421626603653342657,
      "inState": 662315042812135154,
      "keyMapping": {
        "input": [
          {
            "childKey": "lhs",
            "parentKey": "workflow",
            "value": null
          },
          {
            "childKey": "rhs",
            "parentKey": null,
            "value": "adhoc"
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 1127883625959574268,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3906756477846625877,
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
      "id": 2232504332093969980,
      "inState": 1127883625959574268,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 1916846391743147637,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4252726110224364806,
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
      "id": 2139084451508981149,
      "inState": 1916846391743147637,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2634651831492456620,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2613759417509618279,
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
