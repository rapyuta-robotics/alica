{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 4323078219500533488,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 3592474165381227575,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 2102617130958802996,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 3592474165381227575,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "SchedulingPlanInitTermTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 4323078219500533488,
      "id": 2102617130958802996,
      "inTransitions": [],
      "name": "InitPlanState",
      "outTransitions": [
        341681375838380172
      ],
      "parentPlan": 3592474165381227575,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 2127405351341883096,
      "inTransitions": [
        3606040497442350661
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 3592474165381227575,
      "positionWeb": {
        "x": 1202,
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
      "id": 3579842701140903694,
      "inTransitions": [
        593431144432466057
      ],
      "name": "TerminateSubPlanStates",
      "outTransitions": [
        3606040497442350661
      ],
      "parentPlan": 3592474165381227575,
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
          "abstractPlan": "GlobalCounterIncreasePlan.pml#2157924986756626031",
          "comment": "",
          "configuration": null,
          "id": 3292949166536516076,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        },
        {
          "abstractPlan": "GlobalCounterIncreasePlan.pml#2157924986756626031",
          "comment": "",
          "configuration": null,
          "id": 4299573855058001178,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3966925913280437067,
      "inTransitions": [
        341681375838380172
      ],
      "name": "InitSubPlansState",
      "outTransitions": [
        593431144432466057
      ],
      "parentPlan": 3592474165381227575,
      "positionWeb": {
        "x": 686,
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
      "condition": null,
      "id": 341681375838380172,
      "inState": 2102617130958802996,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3966925913280437067,
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
        "id": 685309083377904341,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": null,
      "id": 593431144432466057,
      "inState": 3966925913280437067,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3579842701140903694,
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
        "id": 2241291805664305324,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": null,
      "id": 3606040497442350661,
      "inState": 3579842701140903694,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2127405351341883096,
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
        "id": 886395922626094969,
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
