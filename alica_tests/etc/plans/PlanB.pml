{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1629896055805,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1629896055805",
      "plan": 1629895853508,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1629896057548,
      "successRequired": false,
      "task": "taskrepository.tsk#1613371619454"
    }
  ],
  "frequency": 50,
  "id": 1629895853508,
  "implementationName": "PlanA",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "PlanB",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanBA.pml#1629895873188",
          "comment": "",
          "configuration": null,
          "id": 1629896077656,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1629896077656"
        }
      ],
      "entryPoint": 1629896055805,
      "id": 1629896057548,
      "inTransitions": [],
      "name": "PlanBA",
      "outTransitions": [
        3959155811621906275
      ],
      "parentPlan": 1629895853508,
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
      "id": 3189279155382680298,
      "inTransitions": [
        3959155811621906275
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 1629895853508,
      "positionWeb": {
        "x": 690.5770509105015,
        "y": 203.11082957767024
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
      "condition": "ConditionRepository.cnd#2",
      "id": 3959155811621906275,
      "inState": 1629896057548,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3189279155382680298,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2830866837508311311,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.0,
  "variables": []
}
