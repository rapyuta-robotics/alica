{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1629895952886,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1629895952886",
      "plan": 1629895837159,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1629895956631,
      "successRequired": false,
      "task": "taskrepository.tsk#1613371619454"
    }
  ],
  "frequency": 30,
  "id": 1629895837159,
  "implementationName": "",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "PlanA",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanAA.pml#1629895864090",
          "comment": "",
          "configuration": null,
          "id": 1629895990979,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1629895990979"
        }
      ],
      "entryPoint": 1629895952886,
      "id": 1629895956631,
      "inTransitions": [],
      "name": "PlanAA",
      "outTransitions": [
        246551663950349676
      ],
      "parentPlan": 1629895837159,
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
      "id": 358500327688047702,
      "inTransitions": [
        246551663950349676
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 1629895837159,
      "positionWeb": {
        "x": 607.6655518394649,
        "y": 224.42976588628764
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
      "id": 246551663950349676,
      "inState": 1629895956631,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 358500327688047702,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 778903744121544801,
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
