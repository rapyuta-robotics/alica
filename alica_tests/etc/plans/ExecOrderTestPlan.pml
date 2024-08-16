{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 1592696283737954035,
      "key": "EntryState2PlanAState",
      "type": "bool"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 3081710972011240518,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 3264434119601111942,
      "positionWeb": {
        "x": -11.882591093117409,
        "y": 217.10979432326096
      },
      "state": 1499196400105849381,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 3264434119601111942,
  "implementationName": "UntracedPlan",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "ExecOrderTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanB.pml#1629895853508",
          "comment": "",
          "configuration": null,
          "id": 329278439059332417,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1169226780739813167,
      "inTransitions": [
        1937475276147842179
      ],
      "name": "PlanBState",
      "outTransitions": [
        493677033086680687,
        2746610331342381508
      ],
      "parentPlan": 3264434119601111942,
      "positionWeb": {
        "x": 716,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 3081710972011240518,
      "id": 1499196400105849381,
      "inTransitions": [],
      "name": "EntryState",
      "outTransitions": [
        1613013324428941308
      ],
      "parentPlan": 3264434119601111942,
      "positionWeb": {
        "x": 200,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanA.pml#1629895837159",
          "comment": "",
          "configuration": null,
          "id": 3082238635369072637,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3083796984126971274,
      "inTransitions": [
        1613013324428941308,
        2746610331342381508
      ],
      "name": "PlanAState",
      "outTransitions": [
        1937475276147842179
      ],
      "parentPlan": 3264434119601111942,
      "positionWeb": {
        "x": 458,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 3313013107349346212,
      "inTransitions": [
        493677033086680687
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 3264434119601111942,
      "positionWeb": {
        "x": 974,
        "y": 200
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3677391121921226485",
      "id": 493677033086680687,
      "inState": 1169226780739813167,
      "keyMapping": {
        "input": [
          {
            "childKey": "expected",
            "parentKey": null,
            "value": 81
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3313013107349346212,
      "pointsWeb": [
        {
          "x": 874,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2144060677891213947,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 1613013324428941308,
      "inState": 1499196400105849381,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "EntryState2PlanAState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3083796984126971274,
      "pointsWeb": [
        {
          "x": 358,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2603548172229370814,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2",
      "id": 1937475276147842179,
      "inState": 3083796984126971274,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 1169226780739813167,
      "pointsWeb": [
        {
          "x": 616,
          "y": 29
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1507856381414068358,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2985374601173748991",
      "id": 2746610331342381508,
      "inState": 1169226780739813167,
      "keyMapping": {
        "input": [
          {
            "childKey": "expected",
            "parentKey": null,
            "value": 81
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3083796984126971274,
      "pointsWeb": [
        {
          "x": 616,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1053756108049156514,
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
