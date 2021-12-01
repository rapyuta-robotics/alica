{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 3150793708487666867,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 2252865124432942907,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 2800951832651805821,
      "successRequired": false,
      "task": "taskrepository.tsk#1163169622598227531"
    }
  ],
  "frequency": 0,
  "id": 2252865124432942907,
  "masterPlan": false,
  "name": "DynamicTaskAssignmentTest",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/DynamicTaskBehavior.beh#4044546549214673470",
          "comment": "",
          "configuration": null,
          "id": 454634566206861074,
          "name": ""
        }
      ],
      "entryPoint": 3150793708487666867,
      "id": 2800951832651805821,
      "inTransitions": [],
      "name": "DynamicState1",
      "outTransitions": [
        1813059625501892456
      ],
      "parentPlan": 2252865124432942907,
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
      "id": 2788356913272296281,
      "inTransitions": [
        1813059625501892456
      ],
      "name": "DynamicTaskFinished",
      "outTransitions": [],
      "parentPlan": 2252865124432942907,
      "positionWeb": {
        "x": 686,
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
      "id": 1813059625501892456,
      "inState": 2800951832651805821,
      "name": "",
      "outState": 2788356913272296281,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1078898265232036813,
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