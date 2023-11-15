{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1629896015785,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1629896015785",
      "plan": 1629895864090,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1629896006533,
      "successRequired": false,
      "task": "taskrepository.tsk#1613371619454"
    }
  ],
  "frequency": 40,
  "id": 1629895864090,
  "implementationName": "PlanA",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "PlanAA",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "BehAAA.beh#1629895901559",
          "comment": "",
          "configuration": null,
          "id": 1629896034213,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1629896034213"
        }
      ],
      "entryPoint": 1629896015785,
      "id": 1629896006533,
      "inTransitions": [],
      "name": "BehAAA",
      "outTransitions": [
        2608976802822477412
      ],
      "parentPlan": 1629895864090,
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
      "id": 4002779909425878968,
      "inTransitions": [
        2608976802822477412
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 1629895864090,
      "positionWeb": {
        "x": 624.659977703456,
        "y": 229.02285395763658
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
      "condition": "ConditionRepository.cnd#1",
      "id": 2608976802822477412,
      "inState": 1629896006533,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 4002779909425878968,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 461890677292476260,
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
