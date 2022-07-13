{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1402488903550,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1402488893641,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1402488959965,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1402488893641,
  "inheritBlackboard": false,
  "masterPlan": false,
  "name": "Defend",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/Tackle.beh#1402488939130",
          "comment": "",
          "configuration": null,
          "id": 1587718662824,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662824"
        },
        {
          "abstractPlan": "Tackle.pml#1402489318663",
          "comment": "",
          "configuration": null,
          "id": 1587718662826,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662826"
        }
      ],
      "entryPoint": null,
      "id": 1402488903549,
      "inTransitions": [
        1402488990761
      ],
      "name": "Tackle",
      "outTransitions": [
        1402488991762
      ],
      "parentPlan": 1402488893641,
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
      "id": 1402488910751,
      "inTransitions": [
        1402489071510
      ],
      "name": "GetGoal",
      "outTransitions": [],
      "parentPlan": 1402488893641,
      "positionWeb": {
        "x": 944,
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
      "entryPoint": 1402488903550,
      "id": 1402488959965,
      "inTransitions": [
        1402488991762
      ],
      "name": "GetBall",
      "outTransitions": [
        1402488990761,
        1402489064693
      ],
      "parentPlan": 1402488893641,
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
          "abstractPlan": "PlanType.pty#1402489564599",
          "comment": "",
          "configuration": null,
          "id": 1587718662831,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662831"
        }
      ],
      "entryPoint": null,
      "id": 1402489037735,
      "inTransitions": [
        1402489064693
      ],
      "name": "TryToDefendGoal",
      "outTransitions": [
        1402489071510
      ],
      "parentPlan": 1402488893641,
      "positionWeb": {
        "x": 686,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "conditions/conditions.cnd#1678986049909129132",
      "id": 1402488990761,
      "inState": 1402488959965,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "TackleToGetBall",
      "outState": 1402488903549,
      "pointsWeb": [
        {
          "x": 586,
          "y": 29
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1402488991641,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "conditions/conditions.cnd#1678986049909129132",
      "id": 1402488991762,
      "inState": 1402488903549,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "TackleToGetBall",
      "outState": 1402488959965,
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
        "id": 1402488993122,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "conditions/conditions.cnd#1678986049909129132",
      "id": 1402489064693,
      "inState": 1402488959965,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "GetBallToTryToDefendGoal",
      "outState": 1402489037735,
      "pointsWeb": [
        {
          "x": 586,
          "y": 429
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1402489065962,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "conditions/conditions.cnd#1678986049909129132",
      "id": 1402489071510,
      "inState": 1402489037735,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "TryToDefendGoalToGetGoal",
      "outState": 1402488910751,
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
        "id": 1402489073613,
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
