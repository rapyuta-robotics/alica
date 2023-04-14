{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1629895738193,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1629895738193",
      "plan": 1629895582410,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1629895681520,
      "successRequired": false,
      "task": "taskrepository.tsk#1613371619454"
    }
  ],
  "frequency": 0,
  "id": 1629895582410,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "OrderedSchedulingTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanA.pml#1629895837159",
          "comment": "",
          "configuration": null,
          "id": 1629895930356,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1629895930356"
        }
      ],
      "entryPoint": 1629895738193,
      "id": 1629895681520,
      "inTransitions": [
        1629895768181
      ],
      "name": "PlanA",
      "outTransitions": [
        1629895758611
      ],
      "parentPlan": 1629895582410,
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
          "abstractPlan": "PlanB.pml#1629895853508",
          "comment": "",
          "configuration": null,
          "id": 1629895934743,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1629895934743"
        }
      ],
      "entryPoint": null,
      "id": 1629895684249,
      "inTransitions": [
        1629895758611
      ],
      "name": "PlanB",
      "outTransitions": [
        1629895768181
      ],
      "parentPlan": 1629895582410,
      "positionWeb": {
        "x": 686,
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
      "condition": "ConditionRepository.cnd#682216470625774387",
      "id": 1629895758611,
      "inState": 1629895681520,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "FromPlanATo PlanB",
      "outState": 1629895684249,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1629895758612,
        "name": "1629895758612",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "MISSING_COMMENT",
      "condition": "ConditionRepository.cnd#655002160731734731",
      "id": 1629895768181,
      "inState": 1629895684249,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "FromPlanBTo PlanA",
      "outState": 1629895681520,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1629895768182,
        "name": "1629895768182",
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
