{
  "blackboard": [],
  "comment": "Master plan: run CountUp until the counter is high enough, then succeed.",
  "entryPoints": [
    {
      "comment": "One to two agents take the DefaultTask.",
      "id": 5002,
      "isDynamic": false,
      "maxCardinality": 2,
      "minCardinality": 1,
      "name": "",
      "plan": 5001,
      "positionWeb": { "x": 200, "y": 200 },
      "state": 5003,
      "successRequired": true,
      "task": "TaskRepository.tsk#1002"
    }
  ],
  "frequency": 0,
  "id": 5001,
  "implementationName": "MinimalMaster",
  "inheritBlackboard": false,
  "libraryName": "minimal-alica",
  "masterPlan": true,
  "name": "MinimalMaster",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "Runs the CountUp behaviour.",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "CountUp.beh#4001",
          "comment": "",
          "configuration": null,
          "id": 5006,
          "keyMapping": { "input": [], "output": [] },
          "name": ""
        }
      ],
      "entryPoint": 5002,
      "id": 5003,
      "inTransitions": [],
      "name": "Working",
      "outTransitions": [5005],
      "parentPlan": 5001,
      "positionWeb": { "x": 400, "y": 200 },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "Terminal success state.",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 5004,
      "inTransitions": [5005],
      "name": "Finished",
      "outTransitions": [],
      "parentPlan": 5001,
      "positionWeb": { "x": 650, "y": 200 },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "Working -> Finished, guarded by the CountReached condition.",
      "condition": "ConditionRepository.cnd#3002",
      "id": 5005,
      "inState": 5003,
      "keyMapping": { "input": [], "output": [] },
      "name": "",
      "outState": 5004,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 5007,
        "name": "Working2Finished",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.0,
  "variables": []
}
