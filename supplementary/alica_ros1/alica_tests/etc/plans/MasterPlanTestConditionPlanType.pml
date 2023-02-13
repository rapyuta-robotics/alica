{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1418042656596,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1418042656594,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1418042656595,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1418042656594,
  "inheritBlackboard": false,
  "libraryName": "libalica-tests",
  "masterPlan": true,
  "name": "MasterPlanTestConditionPlanType",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1418042656596,
      "id": 1418042656595,
      "inTransitions": [],
      "name": "Start",
      "outTransitions": [
        1418042682960
      ],
      "parentPlan": 1418042656594,
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
          "abstractPlan": "TestPlanType.pty#1418042702402",
          "comment": "",
          "configuration": null,
          "id": 1587718662539,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662539"
        }
      ],
      "entryPoint": null,
      "id": 1418042674811,
      "inTransitions": [
        1418042682960
      ],
      "name": "Plantype",
      "outTransitions": [],
      "parentPlan": 1418042656594,
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
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#2872265442510628524",
      "id": 1418042682960,
      "inState": 1418042656595,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "MISSING_NAME",
      "outState": 1418042674811,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1418042683692,
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
