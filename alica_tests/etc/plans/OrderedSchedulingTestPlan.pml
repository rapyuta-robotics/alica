{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1629895738193,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1629895738193",
      "plan": 1629895582410,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1629895681520,
      "successRequired": false,
      "task": "taskrepository.tsk#1613371619454"
    }
  ],
  "frequency": 0,
  "id": 1629895582410,
  "implementationName": "UntracedPlan",
  "inheritBlackboard": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "OrderedSchedulingTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanA.pml#1629895837159",
          "comment": "",
          "configuration": null,
          "id": 1629895930356,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1629895930356"
        }
      ],
      "entryPoint": 1629895738193,
      "id": 1629895681520,
      "inTransitions": [
        1629895768181
      ],
      "name": "PlanAState",
      "outTransitions": [
        1629895758611,
        3753727146437722291
      ],
      "parentPlan": 1629895582410,
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
          "abstractPlan": "PlanB.pml#1629895853508",
          "comment": "",
          "configuration": null,
          "id": 1629895934743,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1629895934743"
        }
      ],
      "entryPoint": null,
      "id": 1629895684249,
      "inTransitions": [
        1629895758611
      ],
      "name": "PlanBState",
      "outTransitions": [
        1629895768181,
        694002433690351298
      ],
      "parentPlan": 1629895582410,
      "positionWeb": {
        "x": 686,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 3402337965458468818,
      "inTransitions": [
        200267174688903928
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 1629895582410,
      "positionWeb": {
        "x": 1202,
        "y": 400
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
      "id": 3432579755629291493,
      "inTransitions": [
        694002433690351298,
        3753727146437722291
      ],
      "name": "StringCompareState",
      "outTransitions": [
        200267174688903928
      ],
      "parentPlan": 1629895582410,
      "positionWeb": {
        "x": 944,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "MISSING_COMMENT",
      "condition": null,
      "id": 1629895758611,
      "inState": 1629895681520,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "FromPlanATo PlanB",
      "outState": 1629895684249,
      "pointsWeb": [
        {
          "x": 586,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1629895758612,
        "name": "1629895758612",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "MISSING_COMMENT",
      "condition": null,
      "id": 1629895768181,
      "inState": 1629895684249,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "FromPlanBTo PlanA",
      "outState": 1629895681520,
      "pointsWeb": [
        {
          "x": 586,
          "y": 429
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1629895768182,
        "name": "1629895768182",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": null,
      "id": 200267174688903928,
      "inState": 3432579755629291493,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3402337965458468818,
      "pointsWeb": [
        {
          "x": 1102,
          "y": 429
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 625966142166329558,
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
      "id": 694002433690351298,
      "inState": 1629895684249,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3432579755629291493,
      "pointsWeb": [
        {
          "x": 844,
          "y": 429
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3195499981885855124,
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
      "id": 3753727146437722291,
      "inState": 1629895681520,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3432579755629291493,
      "pointsWeb": [
        {
          "x": 586,
          "y": 29
        },
        {
          "x": 715,
          "y": 29
        },
        {
          "x": 844,
          "y": 29
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4569564929046231736,
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
