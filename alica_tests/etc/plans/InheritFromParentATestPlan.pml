{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 557658953352140847,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 185950610054102790,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1981213401458939423,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 185950610054102790,
  "implementationName": "",
  "inheritBlackboard": true,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "InheritFromParentATestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 2588330984497349278,
      "inTransitions": [
        2268952358136177381
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 185950610054102790,
      "positionWeb": {
        "x": 2492,
        "y": 200
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 3807068659782313209,
      "inTransitions": [
        155545721980803188
      ],
      "name": "BoolCheckState",
      "outTransitions": [
        3674805364839002711
      ],
      "parentPlan": 185950610054102790,
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
      "entryPoint": 557658953352140847,
      "id": 1981213401458939423,
      "inTransitions": [],
      "name": "WaitForChildCheckState",
      "outTransitions": [
        155545721980803188
      ],
      "parentPlan": 185950610054102790,
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
          "abstractPlan": "NotInheritFromParentBTestPlan.pml#2263642900900487682",
          "comment": "",
          "configuration": null,
          "id": 1655089164413261303,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1912512209747628177,
      "inTransitions": [
        2453373586649227929
      ],
      "name": "RunChildState",
      "outTransitions": [
        2268952358136177381
      ],
      "parentPlan": 185950610054102790,
      "positionWeb": {
        "x": 2234,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1473752614955870368,
      "inTransitions": [
        3674805364839002711
      ],
      "name": "DoubleCheckState",
      "outTransitions": [
        3538171417827374918
      ],
      "parentPlan": 185950610054102790,
      "positionWeb": {
        "x": 944,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 737679182618201925,
      "inTransitions": [
        2738668016169155258
      ],
      "name": "StringCheckState",
      "outTransitions": [
        2136399575035608972
      ],
      "parentPlan": 185950610054102790,
      "positionWeb": {
        "x": 1460,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1841589429979206762,
      "inTransitions": [
        2136399575035608972
      ],
      "name": "UIntCheckState",
      "outTransitions": [
        274702503629366242
      ],
      "parentPlan": 185950610054102790,
      "positionWeb": {
        "x": 1718,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 3723520305164992257,
      "inTransitions": [
        274702503629366242
      ],
      "name": "WaitState",
      "outTransitions": [
        2453373586649227929
      ],
      "parentPlan": 185950610054102790,
      "positionWeb": {
        "x": 1976,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 534005728445547023,
      "inTransitions": [
        3538171417827374918
      ],
      "name": "IntCheckState",
      "outTransitions": [
        2738668016169155258
      ],
      "parentPlan": 185950610054102790,
      "positionWeb": {
        "x": 1202,
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
      "id": 2268952358136177381,
      "inState": 1912512209747628177,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2588330984497349278,
      "pointsWeb": [
        {
          "x": 2392,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2699829842384471487,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 2453373586649227929,
      "inState": 3723520305164992257,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "WaitState2RunChildState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 1912512209747628177,
      "pointsWeb": [
        {
          "x": 2134,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1656597672713449931,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#89453017592951088",
      "id": 3674805364839002711,
      "inState": 3807068659782313209,
      "keyMapping": {
        "input": [
          {
            "childKey": "left",
            "parentKey": "boolValueA",
            "value": null
          },
          {
            "childKey": "right",
            "parentKey": null,
            "value": false
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 1473752614955870368,
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
        "id": 2832667233547521716,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 155545721980803188,
      "inState": 1981213401458939423,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "WaitForChildCheckState2BoolCheckState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3807068659782313209,
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
        "id": 1737979141296925600,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#4434919629521912839",
      "id": 2136399575035608972,
      "inState": 737679182618201925,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "stringValueA",
            "value": null
          },
          {
            "childKey": "left",
            "parentKey": null,
            "value": "initString"
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 1841589429979206762,
      "pointsWeb": [
        {
          "x": 1618,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2314934188662122139,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2618711924921632178",
      "id": 2738668016169155258,
      "inState": 534005728445547023,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "intValueA",
            "value": null
          },
          {
            "childKey": "left",
            "parentKey": null,
            "value": 1
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 737679182618201925,
      "pointsWeb": [
        {
          "x": 1360,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2194138094258125255,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#344595839150634454",
      "id": 274702503629366242,
      "inState": 1841589429979206762,
      "keyMapping": {
        "input": [
          {
            "childKey": "left",
            "parentKey": null,
            "value": 1
          },
          {
            "childKey": "right",
            "parentKey": "uintValueA",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3723520305164992257,
      "pointsWeb": [
        {
          "x": 1876,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2786379039662157679,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1644313031322732060",
      "id": 3538171417827374918,
      "inState": 1473752614955870368,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "doubleValueA",
            "value": null
          },
          {
            "childKey": "left",
            "parentKey": null,
            "value": 1.0
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 534005728445547023,
      "pointsWeb": [
        {
          "x": 1102,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3793500998732088231,
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
