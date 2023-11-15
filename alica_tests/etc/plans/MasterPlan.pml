{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1402488437263,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1402488437260,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 1402488437261,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1402488437260,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": true,
  "name": "MasterPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "Attack.beh#1402488848841",
          "comment": "",
          "configuration": null,
          "id": 1587718662710,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662710"
        }
      ],
      "entryPoint": 1402488437263,
      "id": 1402488437261,
      "inTransitions": [],
      "name": "Attack",
      "outTransitions": [
        1402488517667,
        1409218318661
      ],
      "parentPlan": 1402488437260,
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
          "abstractPlan": "Defend.pml#1402488893641",
          "comment": "",
          "configuration": null,
          "id": 1587718662713,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662713"
        }
      ],
      "entryPoint": null,
      "id": 1402488463437,
      "inTransitions": [
        1409218318661
      ],
      "name": "Defend",
      "outTransitions": [],
      "parentPlan": 1402488437260,
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
          "abstractPlan": "GoalPlan.pml#1402488870347",
          "comment": "",
          "configuration": null,
          "id": 1587718662716,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662716"
        }
      ],
      "entryPoint": null,
      "id": 1402488470615,
      "inTransitions": [
        1402488519757
      ],
      "name": "Goal",
      "outTransitions": [
        1402488557864
      ],
      "parentPlan": 1402488437260,
      "positionWeb": {
        "x": 944,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "MidFieldStandard.beh#1402488696205",
          "comment": "",
          "configuration": null,
          "id": 1587718662719,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662719"
        },
        {
          "abstractPlan": "DefendMid.beh#1402488730695",
          "comment": "",
          "configuration": null,
          "id": 1587718662721,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662721"
        },
        {
          "abstractPlan": "MidFieldPlayPlan.pml#1402488770050",
          "comment": "",
          "configuration": null,
          "id": 1587718662723,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662723"
        }
      ],
      "entryPoint": null,
      "id": 1402488477650,
      "inTransitions": [
        1402488517667
      ],
      "name": "MidField",
      "outTransitions": [
        1402488519757
      ],
      "parentPlan": 1402488437260,
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
      "id": 1402488536570,
      "inTransitions": [
        1402488557864
      ],
      "name": "SucGoalState",
      "outTransitions": [],
      "parentPlan": 1402488437260,
      "positionWeb": {
        "x": 1202,
        "y": 400
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
      "condition": "ConditionRepository.cnd#1678986049909129132",
      "id": 1402488517667,
      "inState": 1402488437261,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "AttackToGoal",
      "outState": 1402488477650,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1402488519140,
        "name": "MISSING_NAME",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1678986049909129132",
      "id": 1402488519757,
      "inState": 1402488477650,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "MidFieldToGoal",
      "outState": 1402488470615,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1402488520968,
        "name": "MISSING_NAME",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1678986049909129132",
      "id": 1402488557864,
      "inState": 1402488470615,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "GoalToSucGoal",
      "outState": 1402488536570,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1402488558741,
        "name": "MISSING_NAME",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1678986049909129132",
      "id": 1409218318661,
      "inState": 1402488437261,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "AttackToDefend",
      "outState": 1402488463437,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1409218319990,
        "name": "MISSING_NAME",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.1,
  "variables": []
}
