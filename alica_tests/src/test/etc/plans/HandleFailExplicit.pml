{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1530004915642,
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
  "masterPlan": false,
  "name": "HandleFailExplicit",
  "preCondition": null,
  "relativeDirectory": "",
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
          "abstractPlan": "behaviours/AlwaysFail.beh#1532424188199",
          "comment": "",
          "configuration": null,
          "id": 1587718662553,
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
      "id": 1530004992551,
      "inState": 1530004915641,
      "name": "MISSING_NAME",
      "outState": 1530004973591,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1530004993493,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "AnyChildFail",
      "id": 1530004993680,
      "inState": 1530004973591,
      "name": "MISSING_NAME",
      "outState": 1530004975275,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1530004994611,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "C to D, isset(2)",
      "id": 1532424092280,
      "inState": 1530004975275,
      "name": "MISSING_NAME",
      "outState": 1532424087894,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1532424093178,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "AnyChildFail",
      "id": 1532424112331,
      "inState": 1532424087894,
      "name": "MISSING_NAME",
      "outState": 1532424097662,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1532424113475,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.1,
  "variables": []
}