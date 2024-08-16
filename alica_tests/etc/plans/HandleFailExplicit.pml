{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 4227261568323854459,
      "key": "aToBSwitch",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 4460455267274597988,
      "key": "cToDSwitch",
      "type": "int64"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1530004915642,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1530004915640,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1530004915641,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1530004915640,
  "implementationName": "",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "HandleFailExplicit",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1530004915642,
      "id": 1530004915641,
      "inTransitions": [],
      "name": "A",
      "outTransitions": [
        1530004992551
      ],
      "parentPlan": 1530004915640,
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
          "abstractPlan": "FailsOnOne.pml#1530069246103",
          "comment": "",
          "configuration": null,
          "id": 1587718662548,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662548"
        }
      ],
      "entryPoint": null,
      "id": 1530004973591,
      "inTransitions": [
        1530004992551
      ],
      "name": "B",
      "outTransitions": [
        1530004993680
      ],
      "parentPlan": 1530004915640,
      "positionWeb": {
        "x": 686,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1530004975275,
      "inTransitions": [
        1530004993680
      ],
      "name": "C",
      "outTransitions": [
        1532424092280
      ],
      "parentPlan": 1530004915640,
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
          "abstractPlan": "AlwaysFail.beh#1532424188199",
          "comment": "",
          "configuration": null,
          "id": 1587718662553,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662553"
        }
      ],
      "entryPoint": null,
      "id": 1532424087894,
      "inTransitions": [
        1532424092280
      ],
      "name": "D",
      "outTransitions": [
        1532424112331
      ],
      "parentPlan": 1530004915640,
      "positionWeb": {
        "x": 1202,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1532424097662,
      "inTransitions": [
        1532424112331
      ],
      "name": "E",
      "outTransitions": [],
      "parentPlan": 1530004915640,
      "positionWeb": {
        "x": 1460,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "From A to B, isset(0)",
      "condition": "ConditionRepository.cnd#3787001793582633602",
      "id": 1530004992551,
      "inState": 1530004915641,
      "keyMapping": {
        "input": [
          {
            "childKey": "idx",
            "parentKey": "aToBSwitch",
            "value": null
          }
        ],
        "output": []
      },
      "name": "MISSING_NAME",
      "outState": 1530004973591,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1530004993493,
        "name": "MISSING_NAME",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "AnyChildFail",
      "condition": "ConditionRepository.cnd#711536493236439192",
      "id": 1530004993680,
      "inState": 1530004973591,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "MISSING_NAME",
      "outState": 1530004975275,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1530004994611,
        "name": "MISSING_NAME",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "C to D, isset(2)",
      "condition": "ConditionRepository.cnd#3787001793582633602",
      "id": 1532424092280,
      "inState": 1530004975275,
      "keyMapping": {
        "input": [
          {
            "childKey": "idx",
            "parentKey": "cToDSwitch",
            "value": null
          }
        ],
        "output": []
      },
      "name": "MISSING_NAME",
      "outState": 1532424087894,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1532424093178,
        "name": "MISSING_NAME",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "AnyChildFail",
      "condition": "ConditionRepository.cnd#711536493236439192",
      "id": 1532424112331,
      "inState": 1532424087894,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "MISSING_NAME",
      "outState": 1532424097662,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1532424113475,
        "name": "MISSING_NAME",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.1,
  "variables": []
}
