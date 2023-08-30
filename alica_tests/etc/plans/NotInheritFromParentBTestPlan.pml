{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "id": 53123210794255978,
      "key": "uintValueB",
      "type": "uint64"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 4255008038418301369,
      "key": "doubleValueB",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 3680374252032854739,
      "key": "boolValueB",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 3511659697694726328,
      "key": "intValueB",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 4277380365509580990,
      "key": "stringValueB",
      "type": "std::string"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2306507221779934961,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 2263642900900487682,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 2584628389580830143,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 2263642900900487682,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "NotInheritFromParentBTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 3471316341325581983,
      "inTransitions": [
        1859738224035874603
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 2263642900900487682,
      "positionWeb": {
        "x": 686,
        "y": 200
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "InheritFromParentCTestBeh.beh#3173948008495946955",
          "comment": "",
          "configuration": null,
          "id": 2012965625476563017,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 2306507221779934961,
      "id": 2584628389580830143,
      "inTransitions": [],
      "name": "CheckNotInheritFromParentState",
      "outTransitions": [
        1859738224035874603
      ],
      "parentPlan": 2263642900900487682,
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
      "id": 1859738224035874603,
      "inState": 2584628389580830143,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3471316341325581983,
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
        "id": 1692165902337928048,
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
