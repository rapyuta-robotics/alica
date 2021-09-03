{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1625614705483,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1625614705483",
      "plan": 1625614640417,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 1625614714499,
      "successRequired": false,
      "task": "taskrepository.tsk#1625610762033"
    },
    {
      "comment": "",
      "id": 1625614710816,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1625614710816",
      "plan": 1625614640417,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1625614697742,
      "successRequired": false,
      "task": "taskrepository.tsk#1625610785404"
    }
  ],
  "frequency": 0,
  "id": 1625614640417,
  "masterPlan": false,
  "name": "EngineRulesSchedulingTestPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/EmptyBehaviour.beh#1625610857563",
          "comment": "",
          "configuration": null,
          "id": 1625644679751,
          "name": "1625644679751"
        }
      ],
      "entryPoint": 1625614710816,
      "id": 1625614697742,
      "inTransitions": [],
      "name": "Default Name",
      "outTransitions": [],
      "parentPlan": 1625614640417,
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
          "abstractPlan": "behaviours/EmptyBehaviour.beh#1625610857563",
          "comment": "",
          "configuration": null,
          "id": 1625781179215,
          "name": "1625781179215"
        }
      ],
      "entryPoint": 1625614705483,
      "id": 1625614714499,
      "inTransitions": [],
      "name": "StartEngineRulesSchedulingTest",
      "outTransitions": [
        1625614729978,
        1625776897471
      ],
      "parentPlan": 1625614640417,
      "positionWeb": {
        "x": 428,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/EmptyBehaviour.beh#1625610857563",
          "comment": "",
          "configuration": null,
          "id": 1625644677437,
          "name": "1625644677437"
        }
      ],
      "entryPoint": null,
      "id": 1625614719367,
      "inTransitions": [
        1625614729978
      ],
      "name": "Default Name",
      "outTransitions": [],
      "parentPlan": 1625614640417,
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
      "id": 1625776883489,
      "inTransitions": [
        1625776897471
      ],
      "name": "FailureState",
      "outTransitions": [],
      "parentPlan": 1625614640417,
      "positionWeb": {
        "x": 686,
        "y": 400
      },
      "postCondition": null,
      "success": false,
      "type": "TerminalState",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "MISSING_COMMENT",
      "id": 1625614729978,
      "inState": 1625614714499,
      "name": "FromDefault NameTo Default Name",
      "outState": 1625614719367,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1625614729981,
        "name": "1625614729981",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "MISSING_COMMENT",
      "id": 1625776897471,
      "inState": 1625614714499,
      "name": "FromStartEngineRulesSchedulingTestTo Default Name",
      "outState": 1625776883489,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1625776897472,
        "name": "1625776897472",
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