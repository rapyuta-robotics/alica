{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1664745578583462615,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 523433959791172508,
      "positionWeb": {
        "x": 262,
        "y": 327.625
      },
      "state": 4345760372867415186,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 523433959791172508,
  "implementationName": "UntracedPlan",
  "inheritBlackboard": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "TransitionAfterInitTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1209111954261567822,
      "inTransitions": [
        444990720774939114
      ],
      "name": "",
      "outTransitions": [],
      "parentPlan": 523433959791172508,
      "positionWeb": {
        "x": 662,
        "y": 382.625
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
      "id": 1907664681300098997,
      "inTransitions": [
        1309706687175053027
      ],
      "name": "FailureState",
      "outTransitions": [],
      "parentPlan": 523433959791172508,
      "positionWeb": {
        "x": 500,
        "y": 543.625
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
          "id": 740931476575106816,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 1664745578583462615,
      "id": 4345760372867415186,
      "inTransitions": [],
      "name": "PlanAState",
      "outTransitions": [
        444990720774939114,
        1309706687175053027
      ],
      "parentPlan": 523433959791172508,
      "positionWeb": {
        "x": 417,
        "y": 343.625
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3677391121921226485",
      "id": 444990720774939114,
      "inState": 4345760372867415186,
      "keyMapping": {
        "input": [
          {
            "childKey": "expected",
            "parentKey": null,
            "value": 3
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 1209111954261567822,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2369982338343052723,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#909673041355119516",
      "id": 1309706687175053027,
      "inState": 4345760372867415186,
      "keyMapping": {
        "input": [
          {
            "childKey": "expected",
            "parentKey": null,
            "value": 3
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 1907664681300098997,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 472249793450597841,
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
