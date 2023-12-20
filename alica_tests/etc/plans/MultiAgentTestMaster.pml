{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1413200842975,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1413200842975",
      "plan": 1413200842973,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1413200842974,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1413200842973,
  "implementationName": "",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica-tests",
  "masterPlan": true,
  "name": "MultiAgentTestMaster",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1413200842975,
      "id": 1413200842974,
      "inTransitions": [],
      "name": "Init",
      "outTransitions": [
        1413201226246
      ],
      "parentPlan": 1413200842973,
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
          "abstractPlan": "MultiAgentTestPlan.pml#1413200862180",
          "comment": "",
          "configuration": null,
          "id": 1587718662949,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662949"
        }
      ],
      "entryPoint": null,
      "id": 1413201213955,
      "inTransitions": [
        1413201226246
      ],
      "name": "Start",
      "outTransitions": [
        1413201388722
      ],
      "parentPlan": 1413200842973,
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
          "abstractPlan": "Attack.beh#1402488848841",
          "comment": "",
          "configuration": null,
          "id": 1587718662952,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662952"
        }
      ],
      "entryPoint": null,
      "id": 1413201380359,
      "inTransitions": [
        1413201388722
      ],
      "name": "Finished",
      "outTransitions": [],
      "parentPlan": 1413200842973,
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
      "comment": "",
      "condition": "ConditionRepository.cnd#2208457928613785430",
      "id": 1413201226246,
      "inState": 1413200842974,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "1413201226246",
      "outState": 1413201213955,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1413201227586,
        "name": "1413201227586",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1648591654803570403",
      "id": 1413201388722,
      "inState": 1413201213955,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "1413201388722",
      "outState": 1413201380359,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1413201389955,
        "name": "1413201389955",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.1,
  "variables": []
}
