{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1625614674465,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1625614674465",
      "plan": 1625610679488,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 1625783824098,
      "successRequired": false,
      "task": "taskrepository.tsk#1625614670867"
    }
  ],
  "frequency": 0,
  "id": 1625610679488,
  "masterPlan": true,
  "name": "EngineRulesSchedulingTestMaster",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "EngineRulesSchedulingTestPlan.pml#1625614640417",
          "comment": "",
          "configuration": null,
          "id": 1625614693778,
          "name": "1625614693778"
        }
      ],
      "entryPoint": null,
      "id": 1625614677498,
      "inTransitions": [
        1625783867494
      ],
      "name": "GoIntoSubPlan",
      "outTransitions": [
        1626848015857
      ],
      "parentPlan": 1625610679488,
      "positionWeb": {
        "x": 686,
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
          "id": 1625784437764,
          "name": "1625784437764"
        }
      ],
      "entryPoint": 1625614674465,
      "id": 1625783824098,
      "inTransitions": [],
      "name": "EntryState",
      "outTransitions": [
        1625783867494,
        1625783869824
      ],
      "parentPlan": 1625610679488,
      "positionWeb": {
        "x": 428,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1625783835198,
      "inTransitions": [
        1625783869824
      ],
      "name": "FailureState",
      "outTransitions": [],
      "parentPlan": 1625610679488,
      "positionWeb": {
        "x": 686,
        "y": 400
      },
      "postCondition": null,
      "success": false,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1626848011700,
      "inTransitions": [
        1626848015857
      ],
      "name": "Default Name",
      "outTransitions": [],
      "parentPlan": 1625610679488,
      "positionWeb": {
        "x": 944,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "MISSING_COMMENT",
      "id": 1625783867494,
      "inState": 1625783824098,
      "name": "FromEntryStateTo GoIntoSubPlan",
      "outState": 1625614677498,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1625783867495,
        "name": "1625783867495",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "MISSING_COMMENT",
      "id": 1625783869824,
      "inState": 1625783824098,
      "name": "FromEntryStateTo FailureState",
      "outState": 1625783835198,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1625783869825,
        "name": "1625783869825",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "MISSING_COMMENT",
      "id": 1626848015857,
      "inState": 1625614677498,
      "name": "FromGoIntoSubPlanTo Default Name",
      "outState": 1626848011700,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1626848015861,
        "name": "1626848015861",
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